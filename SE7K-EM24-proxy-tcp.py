#!/usr/bin/env python3
"""SolarEdge SE7K to Carlo Gavazzi EM24 Modbus TCP proxy.

English
-------
This module acts as a Modbus TCP client and a Modbus TCP server at the same
time. As a client it cyclically polls a SolarEdge SE7K inverter, which
provides both its own inverter data and the data of the attached utility
meter (SE-MTR-3Y-400V-A) in SunSpec format. As a server it republishes that
data under two unit IDs:

* Unit ID 1 - the utility meter data, mapped to the register layout of a
  Carlo Gavazzi EM24 energy meter, so that consumers expecting an EM24 can
  read it unchanged.
* Unit ID 2 - the values read from the SE7K, kept in their original SunSpec
  layout (common model, inverter model 103 and meter model 203).

Deutsch
-------
Das Modul arbeitet gleichzeitig als Modbus-TCP-Client und als
Modbus-TCP-Server. Als Client liest es zyklisch einen SolarEdge-SE7K-
Wechselrichter aus, der sowohl seine eigenen Wechselrichterdaten als auch die
Daten des angeschlossenen EVU-Zaehlers (SE-MTR-3Y-400V-A) im SunSpec-Format
bereitstellt. Als Server stellt es diese Daten unter zwei Unit-IDs erneut zur
Verfuegung:

* Unit-ID 1 - die EVU-Zaehlerdaten, abgebildet auf das Registerlayout eines
  Carlo-Gavazzi-EM24-Zaehlers, damit Verbraucher, die einen EM24 erwarten,
  sie unveraendert lesen koennen.
* Unit-ID 2 - die vom SE7K eingelesenen Werte im urspruenglichen
  SunSpec-Format (Common Model, Inverter-Modell 103 und Meter-Modell 203).
"""

import argparse
import configparser
import importlib
import logging
import math
import sys
import threading
import time

from pymodbus.server.sync import StartTcpServer
from pymodbus.server.sync import ModbusTcpServer
from pymodbus.constants import Endian
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.transaction import ModbusSocketFramer
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.datastore import ModbusSlaveContext
from pymodbus.datastore import ModbusServerContext
from pymodbus.payload import BinaryPayloadBuilder


class EM24SlaveContext(ModbusSlaveContext):
    def getValues(self, fx, address, count=1):
        """Liest EM24-Register.

        Wird vom Pymodbus-Server bei jeder Registeranfrage aufgerufen.
        :param fx: Modbus-Funktionscode des Lesezugriffs.
        :param address: Startadresse des angeforderten Registers.
        :param count: Anzahl der zu lesenden Register, standardmaessig 1.
        :return: Liste der Registerwerte.
        """
        if (address == 11 and count==1):
            logger.debug("Gavazzi model number 1648 supplied")
            return [1648]
        return super().getValues(fx, address, count)



class ModbusMyTcpServer(ModbusTcpServer):
    clientCounter={}

    def process_request(self, request, client):
        """Verarbeitet eine Modbus-Anfrage und zaehlt Clientverbindungen.

        Wird von Pymodbus fuer jede eingehende Anfrage aufgerufen.
        :param request: Dekodierte Modbus-Anfrage.
        :param client: Clientadresse als Netzwerkadresse/Tuple.
        :return: Rueckgabewert der Basisklassenimplementierung.
        """
        self.clientCounter[client[0]] = self.clientCounter.get(client[0],0) + 1
        
        logger = logging.getLogger()
        if self.clientCounter[client[0]]%1000 == 1:
            logger.debug("Served client %s, request count=%s, request=%s", client[0], self.clientCounter[client[0]], request)

        super().process_request(request,client)

    def shutdown(self):
        """Beendet den Modbus-Server.

        Wird beim Herunterfahren oder durch Pymodbus aufgerufen.
        :return: Rueckgabewert der Basisklassenimplementierung.
        """
        logger = logging.getLogger()
        logger.info("shutdown to serve client")
        super().shutdown()

    def server_close(self):
        """Schliesst den Modbus-Server.

        Wird nach dem Ende der Server-Schleife von Pymodbus aufgerufen.
        :return: Rueckgabewert der Basisklassenimplementierung.
        """
        logger = logging.getLogger()
        logger.debug("Modbus server stopped")
        super().server_close()


# --------------------------------------------------------------------------- #
# Creation Factorie
# --------------------------------------------------------------------------- #
def StartMyTcpServer(context=None, identity=None, address=None,
                   custom_functions=[], **kwargs):
    """Erzeugt und startet den TCP-Server.

    Wird einmal aus dem Hauptprogramm aufgerufen und blockiert anschliessend
    in der Server-Schleife.
    :param context: Modbus-Datenmodell mit den Slave-Kontexten.
    :param identity: Optionale Modbus-Geraeteidentifikation.
    :param address: Bind-Adresse als `(host, port)`-Tuple.
    :param custom_functions: Optionale Liste zusaetzlicher Modbus-Funktionen.
    :param kwargs: Weitere Optionen fuer den Pymodbus-Server, insbesondere
        der Framer.
    :return: Kehrt erst nach dem Beenden des Servers zurueck.
    """
    framer = kwargs.pop("framer", ModbusSocketFramer)
    server = ModbusMyTcpServer(context, framer, identity, address, **kwargs)

    for f in custom_functions:
        server.decoder.register(f)
    server.serve_forever()


def _line_voltage(phase_a, phase_b):
    """Berechnet die Leiterspannung aus zwei Phasenspannungen.

    Wird von :func:`setMeterValues` fuer die EM24-Leiterspannungen aufgerufen.
    :param phase_a: Erste Phasenspannung als skalierten Registerwert.
    :param phase_b: Zweite Phasenspannung als skalierten Registerwert.
    :return: Gerundeter positiver Leiterspannungswert im Registerformat.
    """
    return int(round(math.sqrt(phase_a ** 2 + phase_b ** 2 + phase_a * phase_b)))


def _scale_factor(values, key):
    scale = values.get(key, 0)
    if scale is None or scale == -32768:
        return 0
    return 10 ** scale


_ACTIVE_POWER_KEYS = ("power_int", "l1_power_int", "l2_power_int", "l3_power_int")


class PowerEwmaFilter:
    """Exponentiell gewichteter gleitender Mittelwert der Wirkleistung.

    Die Victron-Nullregelung wertet ausschliesslich die Wirkleistung aus und
    neigt bei verrauschten Messwerten zum Aufschwingen. Der Filter glaettet
    daher nur diese Werte. Der Glaettungsfaktor wird aus der tatsaechlich
    vergangenen Zykluszeit berechnet, damit die Zeitkonstante unabhaengig von
    Jitter und ausgelassenen Zyklen bleibt.
    """

    def __init__(self, tau):
        """:param tau: Zeitkonstante in Sekunden, <=0 deaktiviert den Filter."""
        self._tau = tau
        self._state = {}
        self._last = None

    def apply(self, values, now):
        """Ersetzt die Wirkleistungs-Rohwerte durch ihre gefilterten Werte.

        Wird in jedem Lesezyklus des Update-Threads aufgerufen.
        :param values: Dictionary mit den Rohwerten des Zaehlers.
        :param now: Zeitstempel des Zyklus aus `time.monotonic()`.
        :return: None; ``values`` wird direkt veraendert.
        """
        if self._tau <= 0:
            return

        # Gefiltert wird in Watt, damit ein Wechsel des SunSpec-Skalenfaktors
        # den Filterzustand nicht verfaelscht.
        scale = _scale_factor(values, "power_scale_int")
        if not scale:
            return

        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        alpha = 1.0 - math.exp(-dt / self._tau)

        for key in _ACTIVE_POWER_KEYS:
            raw = values.get(key)
            if raw is None:
                continue
            sample = raw * scale
            previous = self._state.get(key)
            filtered = sample if previous is None else previous + alpha * (sample - previous)
            self._state[key] = filtered
            values[key] = int(round(filtered / scale))


def _reg(values, key, scale, factor):
    """Rechnet einen SunSpec-Rohwert in einen EM24-Registerwert um.

    Es wird gerundet und nicht abgeschnitten, da die Gleitkommadarstellung
    von ``10 ** scale`` sonst systematisch ein LSB zu wenig liefert.
    :param values: Dictionary mit den Rohwerten des Zaehlers.
    :param key: Schluessel des Rohwerts.
    :param scale: Aus dem SunSpec-Skalenfaktor berechneter Multiplikator.
    :param factor: EM24-Registeraufloesung, z.B. 10 fuer 0,1-Schritte.
    :return: Gerundeter Registerwert als int.
    """
    return int(round(values.get(key, 0) * scale * factor))


def _reg_sum(values, keys, scale, factor):
    """Summiert mehrere Rohwerte und rechnet sie in ein EM24-Register um.

    Wird fuer die EM24-Blindenergie benoetigt, die im SunSpec-Modell auf
    mehrere Quadranten aufgeteilt ist.
    :param values: Dictionary mit den Rohwerten des Zaehlers.
    :param keys: Iterable der zu summierenden Schluessel.
    :param scale: Aus dem SunSpec-Skalenfaktor berechneter Multiplikator.
    :param factor: EM24-Registeraufloesung.
    :return: Gerundeter Registerwert als int.
    """
    return int(round(sum(values.get(key, 0) for key in keys) * scale * factor))


def _log_meter_values(logger, values):
    invalid_scales = [
        key[:-4]
        for key in (
            "energy_apparent_scale_int",
            "energy_reactive_scale_int",
        )
        if values.get(key) in (None, -32768)
    ]
    if invalid_scales:
        logger.debug("Meter: nicht implementierte Skalen: %s", ", ".join(invalid_scales))

    current_scale = _scale_factor(values, "current_scale_int")
    voltage_scale = _scale_factor(values, "voltage_scale_int")
    frequency_scale = _scale_factor(values, "frequency_scale_int")
    power_scale = _scale_factor(values, "power_scale_int")
    apparent_power_scale = _scale_factor(values, "power_apparent_scale_int")
    reactive_power_scale = _scale_factor(values, "power_reactive_scale_int")
    power_factor_scale = _scale_factor(values, "power_factor_scale_int")

    logger.debug(
        "Meter: U_LN=%.2f/%.2f/%.2f V, I=%.3f/%.3f/%.3f A, "
        "P=%.1f W, S=%.1f VA, Q=%.1f var, cosphi=%.3f, f=%.2f Hz",
        values.get("l1n_voltage_int", 0) * voltage_scale,
        values.get("l2n_voltage_int", 0) * voltage_scale,
        values.get("l3n_voltage_int", 0) * voltage_scale,
        values.get("l1_current_int", 0) * current_scale,
        values.get("l2_current_int", 0) * current_scale,
        values.get("l3_current_int", 0) * current_scale,
        values.get("power_int", 0) * power_scale,
        values.get("power_apparent_int", 0) * apparent_power_scale,
        values.get("power_reactive_int", 0) * reactive_power_scale,
        values.get("power_factor_int", 0) * power_factor_scale,
        values.get("frequency_int", 0) * frequency_scale,
    )

    logger.debug(
        "Meter-Energie: Import aktiv=%.2f kWh, Export aktiv=%.2f kWh",
        values.get("import_energy_active_int", 0)
        * _scale_factor(values, "energy_active_scale_int")
        / 1000,
        values.get("export_energy_active_int", 0)
        * _scale_factor(values, "energy_active_scale_int")
        / 1000,
    )


def setMeterValues(values, block):
    """Schreibt EM24-Messwerte in einen Payload.

    Wird bei jeder Messwertaktualisierung aus dem Update-Thread aufgerufen.
    :param values: Dictionary mit Rohwerten und SunSpec-Scale-Faktoren des
        angeschlossenen Zaehlers.
    :param block: BinaryPayloadBuilder, in den die EM24-Register geschrieben
        werden.
    :return: None; der Payload wird direkt veraendert.
    """
    if not values:
        block.add_16bit_uint(0)
        block.add_16bit_uint(0)
        return

    block.add_16bit_uint(1)
    block.add_16bit_uint(65)
    block.add_string    (values.get("c_manufacturer_str"  ,"12345678901234567890123456789012").ljust(32,' '))
    block.add_string    (values.get("c_model_str"         ,"12345678901234567890123456789012").ljust(32,' '))
    block.add_string    (values.get("c_option_str"        ,"1234567890123456").ljust(16,' '))
    block.add_string    (values.get("c_version_str"       ,"1234567890123456").ljust(16,' '))
    block.add_string    (values.get("c_serialnumber_str"  ,"12345678901234567890123456789012").ljust(32,' '))
    block.add_16bit_int (values.get("c_deviceaddress_int" , 0))

    block.add_16bit_int (values.get("c_sunspec_did_int"   , 103))
    block.add_16bit_int (values.get("c_sunspec_length_int", 50))    
    block.add_16bit_uint(values.get("current_int" , 0))
    block.add_16bit_uint(values.get("l1_current_int" , 0))
    block.add_16bit_uint(values.get("l2_current_int" , 0))
    block.add_16bit_uint(values.get("l3_current_int" , 0))
    block.add_16bit_int (values.get("current_scale_int" , 0))

    block.add_16bit_uint(values.get("voltage_ln_int" , 0))
    block.add_16bit_uint(values.get("l1n_voltage_int" , 0))
    block.add_16bit_uint(values.get("l2n_voltage_int" , 0))
    block.add_16bit_uint(values.get("l3n_voltage_int" , 0))
    l1n_voltage = values.get("l1n_voltage_int", 0)
    l2n_voltage = values.get("l2n_voltage_int", 0)
    l3n_voltage = values.get("l3n_voltage_int", 0)
    # Der SE-MTR-3Y-400V-A liefert die Leiterspannungen selbst; nur falls sie
    # fehlen, werden sie aus den Sternspannungen rekonstruiert.
    l12_voltage = values.get("l12_voltage_int") or _line_voltage(l1n_voltage, l2n_voltage)
    l23_voltage = values.get("l23_voltage_int") or _line_voltage(l2n_voltage, l3n_voltage)
    l31_voltage = values.get("l31_voltage_int") or _line_voltage(l3n_voltage, l1n_voltage)
    voltage_ll = values.get("voltage_ll_int") or int(round((l12_voltage + l23_voltage + l31_voltage) / 3))

    block.add_16bit_uint(voltage_ll)
    block.add_16bit_uint(l12_voltage)
    block.add_16bit_uint(l23_voltage)
    block.add_16bit_uint(l31_voltage)
    block.add_16bit_int (values.get("voltage_scale_int" , 0))
    
    block.add_16bit_uint(values.get("frequency_int" , 0))
    block.add_16bit_int (values.get("frequency_scale_int" , 0))

    block.add_16bit_int(values.get("power_int" , 0))
    block.add_16bit_int(values.get("l1_power_int" , 0))
    block.add_16bit_int(values.get("l2_power_int" , 0))
    block.add_16bit_int(values.get("l3_power_int" , 0))
    block.add_16bit_int (values.get("power_scale_int" , 0))

    block.add_16bit_int(values.get("power_apparent_int" , 0))
    block.add_16bit_int(values.get("l1_power_apparent_int" , 0))
    block.add_16bit_int(values.get("l2_power_apparent_int" , 0))
    block.add_16bit_int(values.get("l3_power_apparent_int" , 0))
    block.add_16bit_int (values.get("power_apparent_scale_int" , 0))

    block.add_16bit_int(values.get("power_reactive_int" , 0))
    block.add_16bit_int(values.get("l1_power_reactive_int" , 0))
    block.add_16bit_int(values.get("l2_power_reactive_int" , 0))
    block.add_16bit_int(values.get("l3_power_reactive_int" , 0))
    block.add_16bit_int (values.get("power_reactive_scale_int" , 0))

    block.add_16bit_int(values.get("power_factor_int" , 0))
    block.add_16bit_int(values.get("l1_power_factor_int" , 0))
    block.add_16bit_int(values.get("l2_power_factor_int" , 0))
    block.add_16bit_int(values.get("l3_power_factor_int" , 0))
    block.add_16bit_int (values.get("power_factor_scale_int" , 0))

    block.add_32bit_uint(values.get("export_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l1_export_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l2_export_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l3_export_energy_active_int" , 0))
    block.add_32bit_uint(values.get("import_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l1_import_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l2_import_energy_active_int" , 0))
    block.add_32bit_uint(values.get("l3_import_energy_active_int" , 0))
    block.add_16bit_int (values.get("energy_active_scale_int" , 0))

    block.add_32bit_uint(values.get("export_energy_apparent_int", 0))
    block.add_32bit_uint(values.get("l1_export_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("l2_export_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("l3_export_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("import_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("l1_import_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("l2_import_energy_apparent_int" , 0))
    block.add_32bit_uint(values.get("l3_import_energy_apparent_int" , 0))
    block.add_16bit_int (values.get("energy_apparent_scale_int" , 0))

    block.add_32bit_uint(values.get("import_energy_reactive_q1_int" , 0))
    block.add_32bit_uint(values.get("l1_import_energy_reactive_q1_int" , 0))
    block.add_32bit_uint(values.get("l2_import_energy_reactive_q1_int" , 0))
    block.add_32bit_uint(values.get("l3_import_energy_reactive_q1_int" , 0))
    block.add_32bit_uint(values.get("import_energy_reactive_q2_int" , 0))
    block.add_32bit_uint(values.get("l1_import_energy_reactive_q2_int" , 0))
    block.add_32bit_uint(values.get("l2_import_energy_reactive_q2_int" , 0))
    block.add_32bit_uint(values.get("l3_import_energy_reactive_q2_int" , 0))
    block.add_32bit_uint(values.get("export_energy_reactive_q3_int" , 0))
    block.add_32bit_uint(values.get("l1_export_energy_reactive_q3_int" , 0))
    block.add_32bit_uint(values.get("l2_export_energy_reactive_q3_int" , 0))
    block.add_32bit_uint(values.get("l3_export_energy_reactive_q3_int" , 0))
    block.add_32bit_uint(values.get("export_energy_reactive_q4_int" , 0))
    block.add_32bit_uint(values.get("l1_export_energy_reactive_q4_int" , 0))
    block.add_32bit_uint(values.get("l2_export_energy_reactive_q4_int" , 0))
    block.add_32bit_uint(values.get("l3_export_energy_reactive_q4_int" , 0))
    block.add_16bit_int (values.get("energy_reactive_scale_int" , 0))

    block.add_32bit_uint(values.get("events_int" , 0))
    #


def setBatteryValues(values, block):
    """Schreibt Batterie-Modelldaten in einen Payload.

    Ist derzeit eine ungenutzte Erweiterung und wird von keinem aktiven
    Pfad aufgerufen.
    :param values: Dictionary mit Batterie-Rohwerten oder ein leerer Wert.
    :param block: BinaryPayloadBuilder fuer die Batterie-Register.
    :return: None; der Payload wird direkt veraendert.
    """
    if not values:
        block.add_16bit_uint(0)
        block.add_16bit_uint(0)
        return

    block.add_16bit_uint(1)    ## TODO set correct values
    block.add_16bit_uint(65)   ## TODO set correct values

def t_update_se7k(ctx, values):
    """Schreibt SE7K-Messwerte in die Register.

    Wird von :func:`t_update` in jedem Aktualisierungszyklus aufgerufen.
    :param ctx: ModbusSlaveContext des SE7K-Zielmodells.
    :param values: Dictionary mit Inverterwerten, Rohwerten und Scale-Faktoren.
    :return: True bei erfolgreicher Aktualisierung, sonst False.
    """

    logger = logging.getLogger()

    try:
        if not values:
            logger.debug("No values read from device; update discarded")
            return False
        
        if values.get("power_ac_int") == 0:
            logger.debug("power_ac_int is zero")
        if values.get("energy_total_int") == 0:
            logger.debug("energy_total_int is zero")
        if values.get("frequency_int") == 0:
            logger.debug("frequency_int is zero; update may be invalid")

        block_40000 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        block_40000.add_string("SunS") 
        block_40000.add_16bit_int(1)
        block_40000.add_16bit_int (values.get("C_SunSpec_Length_int", 65))
        block_40000.add_string    ((values.get("c_manufacturer_str","") + "-truenas").ljust(32,' '))
        block_40000.add_string    (values.get("c_model_str"         ,"12345678901234567890123456789012").ljust(32,' '))
        block_40000.add_string    ("NOT_IMPLEMENTED.".ljust(16,' '))
        block_40000.add_string    (values.get("c_version_str"       ,"1234567890123456").ljust(16,' '))
        block_40000.add_string    (values.get("c_serialnumber_str"  ,"12345678901234567890123456789012").ljust(32,' '))
        block_40000.add_16bit_int (values.get("c_deviceaddress_int" , 0))

        block_40000.add_16bit_int (values.get("c_sunspec_did_int"   , 103))
        block_40000.add_16bit_int (50)
        block_40000.add_16bit_uint(values.get("current_int" , 0))
        block_40000.add_16bit_uint(values.get("l1_current_int" , 0))
        block_40000.add_16bit_uint(values.get("l2_current_int" , 0))
        block_40000.add_16bit_uint(values.get("l3_current_int" , 0))
        block_40000.add_16bit_int (values.get("current_scale_int" , 0))

        block_40000.add_16bit_uint(values.get("l1_voltage_int" , 0))
        block_40000.add_16bit_uint(values.get("l2_voltage_int" , 0))
        block_40000.add_16bit_uint(values.get("l3_voltage_int" , 0))
        block_40000.add_16bit_uint(values.get("l1n_voltage_int" , 0))
        block_40000.add_16bit_uint(values.get("l2n_voltage_int" , 0))
        block_40000.add_16bit_uint(values.get("l3n_voltage_int" , 0))
        block_40000.add_16bit_int (values.get("voltage_scale_int" , 0))
        
        block_40000.add_16bit_int(values.get("power_ac_int" , 0))
        block_40000.add_16bit_int (values.get("power_ac_scale_int" , 0))

        block_40000.add_16bit_uint(values.get("frequency_int" , 0))
        block_40000.add_16bit_int (values.get("frequency_scale_int" , 0))

        block_40000.add_16bit_int(values.get("power_apparent_int" , 0))
        block_40000.add_16bit_int (values.get("power_apparent_scale_int" , 0))

        block_40000.add_16bit_int(values.get("power_reactive_int" , 0))
        block_40000.add_16bit_int (values.get("power_reactive_scale_int" , 0))

        block_40000.add_16bit_int(values.get("power_factor_int" , 0))
        block_40000.add_16bit_int (values.get("power_factor_scale_int" , 0))

        block_40000.add_32bit_uint(values.get("energy_total_int" , 0))
        block_40000.add_16bit_int (values.get("energy_total_scale_int" , 0))

        block_40000.add_16bit_uint(values.get("current_dc_int" , 0))
        block_40000.add_16bit_int (values.get("current_dc_scale_int" , 0))

        block_40000.add_16bit_uint(values.get("voltage_dc_int" , 0))
        block_40000.add_16bit_int (values.get("voltage_dc_scale_int" , 0))

        block_40000.add_16bit_int(values.get("power_dc_int" , 0))
        block_40000.add_16bit_int (values.get("power_dc_scale_int" , 0))

        block_40000.add_16bit_int(0)  # 1 dummy word

        block_40000.add_16bit_int(values.get("temperature_int" , 0))
        block_40000.add_16bit_int(values.get("temperature_scale_int" , 0))

        block_40000.add_16bit_int(0)  # 1 dummy word
        block_40000.add_16bit_int(0)  # 1 dummy word

        block_40000.add_16bit_uint(values.get("status_int" , 0))
        block_40000.add_16bit_uint(values.get("vendor_status_int" , 0))

        block_40000.add_16bit_uint(values.get("rrcr_state_int" , 0))
        block_40000.add_16bit_int(values.get("active_power_limit_int" , 0))
        block_40000.add_32bit_float(values.get("cosphi" , 0))

        block_40000.add_string("123456789012345678901234") # 12 dummy words = 24 bytes
        ctx.setValues(3, 40000, block_40000.to_registers())

        block_40121 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        setMeterValues(values["connected_meters"]["Meter1"], block_40121)
        ctx.setValues(3, 40121, block_40121.to_registers())

        # block_40295 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        # ctx.setValues(3, 40295, block_40295.to_registers())
        # block_40469 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        # ctx.setValues(3, 40469, block_40469.to_registers())

        # block_57598 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        # ctx.setValues(3, 57598, block_57598.to_registers())
        # block_57854 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
        # ctx.setValues(3, 57854, block_57854.to_registers())
    except Exception as e:
        logger.critical(f"SE7K update failed: {e}")
        return False
    return True




def t_update(ctx, SE7K_CTX, stop, module, device, refresh, full_refresh, power_filter_tau=0.0):
    """Liest Messwerte und aktualisiert beide Zielmodelle.

    Wird als Hintergrund-Thread gestartet und laeuft, bis ``stop`` gesetzt
    wird. In jedem Zyklus wird nur die Wirkleistung nachgelesen, da die
    Victron-Nullregelung ausschliesslich diese auswertet und auf Totzeit mit
    Aufschwingen reagiert. Der vollstaendige Registersatz inklusive SE7K-Modell
    wird nur im Abstand von ``full_refresh`` gelesen.
    :param ctx: ModbusSlaveContext des EM24-Zielmodells.
    :param SE7K_CTX: ModbusSlaveContext des SE7K-Zielmodells.
    :param stop: threading.Event zum kontrollierten Beenden des Threads.
    :param module: Geraetemodul mit einer `values(device)`-Funktion.
    :param device: Initialisiertes Geraeteobjekt fuer das Geraetemodul.
    :param refresh: Zykluszeit des schnellen Wirkleistungspfads in Sekunden.
    :param full_refresh: Abstand der vollstaendigen Lesezyklen in Sekunden.
    :param power_filter_tau: Zeitkonstante des EWMA-Wirkleistungsfilters in
        Sekunden, 0 deaktiviert die Glaettung.
    :return: Kehrt normalerweise erst nach dem Setzen von ``stop`` zurueck.
    """

    this_t = threading.currentThread()
    logger = logging.getLogger()

    fast_read = getattr(module, "power_values", None)
    if fast_read is None:
        full_refresh = 0.0

    power_filter = PowerEwmaFilter(power_filter_tau)
    values = {}
    next_full = 0.0

    while not stop.is_set():
        cycle_start = time.monotonic()
        try:
            if not values or cycle_start >= next_full:
                full_values = module.values(device)
                if not full_values:
                    logger.debug(f"{this_t.name}: no new values")
                    continue

                if not t_update_se7k(SE7K_CTX, full_values):
                    logger.debug("SE7K register update was not applied")

                # use the values from the SE-MTR-3Y-400V-A SE Meter
                values = full_values["connected_meters"]["Meter1"]
                next_full = cycle_start + full_refresh
            else:
                values.update(fast_read(device))

            # Nur das EM24-Modell wird geglaettet, das SE7K-Modell oben bleibt roh.
            power_filter.apply(values, time.monotonic())

            if logger.isEnabledFor(logging.DEBUG):
                _log_meter_values(logger, values)


            # apply SunSpec scale factors to convert raw ints to real-world units
            power_scale = _scale_factor(values, 'power_scale_int')
            power_apparent_scale = _scale_factor(values, 'power_apparent_scale_int')
            power_reactive_scale = _scale_factor(values, 'power_reactive_scale_int')
            current_scale = _scale_factor(values, 'current_scale_int')
            voltage_scale = _scale_factor(values, 'voltage_scale_int')
            frequency_scale = _scale_factor(values, 'frequency_scale_int')
            power_factor_scale = _scale_factor(values, 'power_factor_scale_int')
            energy_active_scale = _scale_factor(values, 'energy_active_scale_int')
            energy_reactive_scale = _scale_factor(values, 'energy_reactive_scale_int')

            # EM24 kvarh(+/-) TOT entsprechen den Quadrantensummen des SunSpec-Zaehlers
            import_energy_reactive = _reg_sum(
                values,
                ('import_energy_reactive_q1_int', 'import_energy_reactive_q2_int'),
                energy_reactive_scale, 0.01)
            export_energy_reactive = _reg_sum(
                values,
                ('export_energy_reactive_q3_int', 'export_energy_reactive_q4_int'),
                energy_reactive_scale, 0.01)

            block_0 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
            block_0.add_32bit_int(_reg(values, 'l1n_voltage_int', voltage_scale, 10)) # 0x0000 l1-n voltage  *10
            block_0.add_32bit_int(_reg(values, 'l2n_voltage_int', voltage_scale, 10)) # 0x0002 l2-n voltage
            block_0.add_32bit_int(_reg(values, 'l3n_voltage_int', voltage_scale, 10)) # 0x0004 l3-n voltage
            block_0.add_32bit_int(_reg(values, 'l12_voltage_int', voltage_scale, 10)) # 0x0006 l1-l2 voltage
            block_0.add_32bit_int(_reg(values, 'l23_voltage_int', voltage_scale, 10)) # 0x0008 l2-l3 voltage
            block_0.add_32bit_int(_reg(values, 'l31_voltage_int', voltage_scale, 10)) # 0x000A l3-l1 voltage
            block_0.add_32bit_int(abs(_reg(values, 'l1_current_int', current_scale, 1000))) # 0x000C current l1  *1000
            block_0.add_32bit_int(abs(_reg(values, 'l2_current_int', current_scale, 1000))) # 0x000E current l2
            block_0.add_32bit_int(abs(_reg(values, 'l3_current_int', current_scale, 1000))) # 0x0010 current l3
            block_0.add_32bit_int(_reg(values, 'l1_power_int', power_scale, -10)) # 0x0012 power l1   *10
            block_0.add_32bit_int(_reg(values, 'l2_power_int', power_scale, -10)) # 0x0014 power l2
            block_0.add_32bit_int(_reg(values, 'l3_power_int', power_scale, -10)) # 0x0016 power l3
            block_0.add_32bit_int(_reg(values, 'l1_power_apparent_int', power_apparent_scale, -10)) # 0x0018 apparent power l1   *10
            block_0.add_32bit_int(_reg(values, 'l2_power_apparent_int', power_apparent_scale, -10)) # 0x001A apparent power l2
            block_0.add_32bit_int(_reg(values, 'l3_power_apparent_int', power_apparent_scale, -10)) # 0x001C apparent power l3
            block_0.add_32bit_int(_reg(values, 'l1_power_reactive_int', power_reactive_scale, -10)) # 0x001E reactive power l1   *10
            block_0.add_32bit_int(_reg(values, 'l2_power_reactive_int', power_reactive_scale, -10)) # 0x0020 reactive power l2
            block_0.add_32bit_int(_reg(values, 'l3_power_reactive_int', power_reactive_scale, -10)) # 0x0022 reactive power l3
            block_0.add_32bit_int(_reg(values, 'voltage_ln_int', voltage_scale, 10)) # 0x0024 l-n voltage  *10
            block_0.add_32bit_int(_reg(values, 'voltage_ll_int', voltage_scale, 10)) # 0x0026 l-l voltage
            block_0.add_32bit_int(_reg(values, 'power_int', power_scale, -10)) # 0x0028 total power              *10
            block_0.add_32bit_int(_reg(values, 'power_apparent_int', power_apparent_scale, -10)) # 0x002A total apparent power
            block_0.add_32bit_int(_reg(values, 'power_reactive_int', power_reactive_scale, -10)) # 0x002C total reactive power
            block_0.add_16bit_int(_reg(values, 'l1_power_factor_int', power_factor_scale, 10)) # 0x002E power factor l1  *1000
            block_0.add_16bit_int(_reg(values, 'l2_power_factor_int', power_factor_scale, 10)) # 0x002F power factor l2
            block_0.add_16bit_int(_reg(values, 'l3_power_factor_int', power_factor_scale, 10)) # 0x0030 power factor l3
            block_0.add_16bit_int(_reg(values, 'power_factor_int', power_factor_scale, 10)) # 0x0031 power factor
            block_0.add_16bit_int(0) # 0x0032 Value –1 correspond to L1-L3-L2 sequence, value 0 correspond to L1-L2-L3 sequence (this value is meaningful only in case of 3-phase systems)

            block_0.add_16bit_uint(_reg(values, 'frequency_int', frequency_scale, 10)) # 0x0033 line frequency  *10

            block_0.add_32bit_int(_reg(values, 'import_energy_active_int', energy_active_scale, 0.01)) # 0x0034 kWh(+) TOT
            block_0.add_32bit_int(import_energy_reactive) # 0x0036 kvarh(+) TOT
            block_0.add_32bit_int(56) # 0x0038 demand power
            block_0.add_32bit_int(58) # 0x003A maximum demand power
            block_0.add_32bit_int(_reg(values, 'import_energy_active_int', energy_active_scale, 0.01)) # 0x003C kWh(+) PAR
            block_0.add_32bit_int(import_energy_reactive) # 0x003E kvarh(+) PAR
            block_0.add_32bit_int(_reg(values, 'l1_import_energy_active_int', energy_active_scale, 0.01)) # 0x0040 kWh(+) L1
            block_0.add_32bit_int(_reg(values, 'l2_import_energy_active_int', energy_active_scale, 0.01)) # 0x0042 kWh(+) L2
            block_0.add_32bit_int(_reg(values, 'l3_import_energy_active_int', energy_active_scale, 0.01)) # 0x0044 kWh(+) L3
            block_0.add_32bit_int(10) # 0x0046 total active energy Tarif 1
            block_0.add_32bit_int(20) # 0x0048 total active energy Tarif 2
            block_0.add_32bit_int(30) # 0x004A total active energy Tarif 3
            block_0.add_32bit_int(40) # 0x004C total active energy Tarif 4
            block_0.add_32bit_int(_reg(values, 'export_energy_active_int', energy_active_scale, 0.01)) # 0x004E kWh(-) TOT
            block_0.add_32bit_int(export_energy_reactive) # 0x0050 kvarh(-) TOT
            block_0.add_32bit_int(2400) # 0x0052 hour                                                       *100
            block_0.add_32bit_int(11) # 0x0054 total reactive energy Tarif 1                                *10
            block_0.add_32bit_int(22) # 0x0056 total reactive energy Tarif 2
            block_0.add_32bit_int(33) # 0x0058 total reactive energy Tarif 3
            block_0.add_32bit_int(44) # 0x005A total reactive energy Tarif 4
            block_0.add_32bit_int(118) # 0x005C apparent demand power
            block_0.add_32bit_int(120) # 0x005E apparent demand power max
            block_0.add_32bit_int(122) # 0x0060 DMD A max                        *10
            ctx.setValues(3, 0, block_0.to_registers())
            ctx.setValues(4, 0, block_0.to_registers())

            block_254 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
            block_254.add_32bit_int(2400) # hour    *100                                                         *100
            block_254.add_32bit_int(256)  # unused                                                       *100
            block_254.add_32bit_int(_reg(values, 'voltage_ln_int', voltage_scale, 10)) # l-n voltage  *10
            block_254.add_32bit_int(_reg(values, 'voltage_ll_int', voltage_scale, 10)) # l-l voltage
            block_254.add_32bit_int(_reg(values, 'power_int', power_scale, -10)) # total power              *10
            block_254.add_32bit_int(_reg(values, 'power_apparent_int', power_apparent_scale, -10)) # total apparent power
            block_254.add_32bit_int(_reg(values, 'power_reactive_int', power_reactive_scale, -10)) # total reactive power
            block_254.add_32bit_int(_reg(values, 'power_factor_int', power_factor_scale, 10)) # power factor
            block_254.add_32bit_int(0) # Value –1 correspond to L1-L3-L2 sequence, value 0 correspond to L1-L2-L3 sequence (this value is meaningful only in case of 3-phase systems)
            block_254.add_32bit_int(_reg(values, 'frequency_int', frequency_scale, 10)) # line frequency  *10
            block_254.add_32bit_int(_reg(values, 'import_energy_active_int', energy_active_scale, 0.01)) # kWh(+) TOT
            block_254.add_32bit_int(import_energy_reactive) # kvarh(+) TOT
            block_254.add_32bit_int(_reg(values, 'export_energy_active_int', energy_active_scale, 0.01)) # kWh(-) TOT
            block_254.add_32bit_int(export_energy_reactive) # kvarh(-) TOT
            block_254.add_32bit_int(56) # demand power
            block_254.add_32bit_int(58) # maximum demand power




            block_254.add_32bit_int(_reg(values, 'l12_voltage_int', voltage_scale, 10)) # l1-l2 voltage
            block_254.add_32bit_int(_reg(values, 'l1n_voltage_int', voltage_scale, 10)) # l1-n voltage  *10
            block_254.add_32bit_int(abs(_reg(values, 'l1_current_int', current_scale, 1000))) # current l1  *1000
            block_254.add_32bit_int(_reg(values, 'l1_power_int', power_scale, -10)) # power l1   *10
            block_254.add_32bit_int(_reg(values, 'l1_power_apparent_int', power_apparent_scale, -10)) # apparent power l1   *10
            block_254.add_32bit_int(_reg(values, 'l1_power_reactive_int', power_reactive_scale, -10)) # reactive power l1   *10
            block_254.add_32bit_int(_reg(values, 'l1_power_factor_int', power_factor_scale, 10)) # power factor l1  *1000

            block_254.add_32bit_int(_reg(values, 'l23_voltage_int', voltage_scale, 10)) # l2-l3 voltage
            block_254.add_32bit_int(_reg(values, 'l2n_voltage_int', voltage_scale, 10)) # l2-n voltage
            block_254.add_32bit_int(abs(_reg(values, 'l2_current_int', current_scale, 1000))) # current l2
            block_254.add_32bit_int(_reg(values, 'l2_power_int', power_scale, -10)) # power l2
            block_254.add_32bit_int(_reg(values, 'l2_power_apparent_int', power_apparent_scale, -10)) # apparent power l2
            block_254.add_32bit_int(_reg(values, 'l2_power_reactive_int', power_reactive_scale, -10)) # reactive power l2
            block_254.add_32bit_int(_reg(values, 'l2_power_factor_int', power_factor_scale, 10)) # power factor l2

            block_254.add_32bit_int(_reg(values, 'l31_voltage_int', voltage_scale, 10)) # l3-l1 voltage
            block_254.add_32bit_int(_reg(values, 'l3n_voltage_int', voltage_scale, 10)) # l3-n voltage
            block_254.add_32bit_int(abs(_reg(values, 'l3_current_int', current_scale, 1000))) # current l3
            block_254.add_32bit_int(_reg(values, 'l3_power_int', power_scale, -10)) # power l3
            block_254.add_32bit_int(_reg(values, 'l3_power_apparent_int', power_apparent_scale, -10)) # apparent power l3
            block_254.add_32bit_int(_reg(values, 'l3_power_reactive_int', power_reactive_scale, -10)) # reactive power l3
            block_254.add_32bit_int(_reg(values, 'l3_power_factor_int', power_factor_scale, 10)) # power factor l3

            block_254.add_32bit_int(0) # Value –1 correspond to L1-L3-L2 sequence, value 0 correspond to L1-L2-L3 sequence (this value is meaningful only in case of 3-phase systems)

            block_254.add_32bit_int(_reg(values, 'import_energy_active_int', energy_active_scale, 0.01)) # kWh(+) PAR
            block_254.add_32bit_int(import_energy_reactive) # kvarh(+) PAR
            block_254.add_32bit_int(_reg(values, 'l1_import_energy_active_int', energy_active_scale, 0.01)) # kWh(+) L1
            block_254.add_32bit_int(_reg(values, 'l2_import_energy_active_int', energy_active_scale, 0.01)) # kWh(+) L2
            block_254.add_32bit_int(_reg(values, 'l3_import_energy_active_int', energy_active_scale, 0.01)) # kWh(+) L3
            block_254.add_32bit_int(10) # total active energy Tarif 1
            block_254.add_32bit_int(20) # total active energy Tarif 2
            block_254.add_32bit_int(30) # total active energy Tarif 3
            block_254.add_32bit_int(40) # total active energy Tarif 4
            block_254.add_32bit_int(346)  # unused                                                       *100
            block_254.add_32bit_int(348)  # unused                                                       *100
            block_254.add_32bit_int(350)  # unused                                                       *100
            block_254.add_32bit_int(352)  # unused                                                       *100
            block_254.add_32bit_int(11) # total reactive energy Tarif 1                                      *10
            block_254.add_32bit_int(22) # total reactive energy Tarif 2
            block_254.add_32bit_int(33) # total reactive energy Tarif 3
            block_254.add_32bit_int(44) # total reactive energy Tarif 4
            block_254.add_32bit_int(262)  # unused                                                       *100
            block_254.add_32bit_int(264)  # unused                                                       *100
            block_254.add_32bit_int(266)  # unused                                                       *100
            block_254.add_32bit_int(268)  # unused                                                       *100
            block_254.add_32bit_int(270)  # unused                                                       *100
            block_254.add_32bit_int(272)  # unused                                                       *100
            block_254.add_32bit_int(274)  # unused                                                       *100
            block_254.add_32bit_int(276)  # unused                                                       *100
            block_254.add_32bit_int(118) # apparent demand power
            block_254.add_32bit_int(120) # apparent demand power max
            block_254.add_32bit_int(122) # DMD A max                        *10
            ctx.setValues(3, 254, block_254.to_registers())
            ctx.setValues(4, 254, block_254.to_registers())

            ## unused values
            # "energy_reactive"              # total reactive energy
            # "l1_export_energy_active", 0)) # exported energy l1
            # "l2_export_energy_active", 0)) # exported energy l2
            # "l3_export_energy_active", 0)) # exported energy l3
            # "l1_energy_reactive", 0)) # reactive energy l1
            # "l2_energy_reactive", 0)) # reactive energy l2
            # "l3_energy_reactive", 0)) # reactive energy l3
            # "l1_energy_apparent", 0)) # apparent energy l1
            # "l2_energy_apparent", 0)) # apparent energy l2
            # "l3_energy_apparent", 0)) # apparent energy l3
            # "minimum_demand_power_active", 0)) # minimum demand power
            # "l1_demand_power_active", 0)) # demand power l1
            # "l2_demand_power_active", 0)) # demand power l2
            # "l3_demand_power_active", 0)) # demand power l3
        except Exception as e:
            logger.critical(f"{this_t.name}: {e}")
        finally:
            elapsed = time.monotonic() - cycle_start
            logger.debug(f"{this_t.name}: cycle took {elapsed:.3f} s")
            # Zykluszeit einhalten, statt die Lesedauer zusaetzlich zu warten
            time.sleep(max(0.0, refresh - elapsed))


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-c", "--config", type=str, default="semp-tcp.conf")
    argparser.add_argument("-v", "--verbose", action="store_true", default=False)
    args = argparser.parse_args()

    default_config = {
        "server": {
            "address": "0.0.0.0",
            "port": 502,
            "framer": "socket",
            "log_level": "INFO",
            "meters": 'Meter1'
        },
        "meters": {
            "dst_address": 2,
            "type": "generic",
            "ct_current": 5,
            "ct_inverted": 0,
            "phase_offset": 120,
            "serial_number": 987654,
            "refresh_rate": 0.5,
            "full_refresh_rate": 5,
            "power_filter_tau": 2.0
        }
    }

    confparser = configparser.ConfigParser()
    confparser.read(args.config)

    if not confparser.has_section("server"):
        confparser["server"] = default_config["server"]

    log_handler = logging.StreamHandler(sys.stdout)
    log_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))

    logger = logging.getLogger()
    logger.setLevel(getattr(logging, confparser["server"].get("log_level", fallback=default_config["server"]["log_level"]).upper()))
    logger.addHandler(log_handler)
    logging.getLogger("pymodbus").setLevel(logging.INFO)

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    slaves = {}
    threads = []
    thread_stops = []

    try:
        if confparser.has_option("server", "meters"):
            meters = [m.strip() for m in confparser["server"].get("meters", fallback=default_config["server"]["meters"]).split(',')]

            for meter in meters:
                address = confparser[meter].getint("dst_address", fallback=default_config["meters"]["dst_address"])
                meter_type = confparser[meter].get("type", fallback=default_config["meters"]["type"])
                meter_module = importlib.import_module(f"devices.{meter_type}")
                meter_device = meter_module.device(confparser[meter])

                EM24_slave_ctx = EM24SlaveContext()
                SE7K_slave_ctx = ModbusSlaveContext()

                # block_11 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                # block_11.add_16bit_int(1648)
                # EM24_slave_ctx.setValues(3, 11, block_11.to_registers())
                # EM24_slave_ctx.setValues(4, 11, block_11.to_registers())

                block_0 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_0.add_32bit_int(1234)
                EM24_slave_ctx.setValues(3, 0, block_0.to_registers())
                EM24_slave_ctx.setValues(4, 0, block_0.to_registers())

                block_770 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_770.add_16bit_int(4126) # Version and revision measurment module
                block_770.add_16bit_int(68)   # 
                block_770.add_16bit_int(4127) # Version and revision communication module
                block_770.add_16bit_int(67)   # 
                block_770.add_16bit_int(0)    # Current tariff 
                EM24_slave_ctx.setValues(3, 770, block_770.to_registers())
                EM24_slave_ctx.setValues(4, 770, block_770.to_registers())

                block_848 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_848.add_16bit_int(4128) # Measurement module’s firmware CRC
                EM24_slave_ctx.setValues(3, 848, block_848.to_registers())
                EM24_slave_ctx.setValues(4, 848, block_848.to_registers())

                block_20480 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_20480.add_string("MB24DINAV23XE1X") 
                EM24_slave_ctx.setValues(3, 20480, block_20480.to_registers())
                EM24_slave_ctx.setValues(4, 20480, block_20480.to_registers())

                block_41216 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_41216.add_16bit_int(3) # Front selector status
                EM24_slave_ctx.setValues(3, 41216, block_41216.to_registers())
                EM24_slave_ctx.setValues(4, 41216, block_41216.to_registers())

                block_4096 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_4096.add_16bit_int(9999) # PASSWORD
                block_4096.add_16bit_int(0)     # unused
                block_4096.add_16bit_int(0)    # Measuring system
                block_4096.add_32bit_int(10)   # Current transformer ratio
                block_4096.add_32bit_int(10)   # Voltage transformer ratio 
                block_4096.add_16bit_int(1)     # unused
                block_4096.add_16bit_int(2)     # unused
                block_4096.add_16bit_int(3)     # unused
                block_4096.add_16bit_int(4)     # unused
                block_4096.add_16bit_int(5)     # unused
                block_4096.add_16bit_int(6)     # unused
                block_4096.add_16bit_int(7)     # unused
                block_4096.add_16bit_int(8)     # unused
                block_4096.add_16bit_int(9)     # unused
                block_4096.add_32bit_int(15)   # Interval time 
                EM24_slave_ctx.setValues(3, 4096, block_4096.to_registers())
                EM24_slave_ctx.setValues(4, 4096, block_4096.to_registers())

                block_4360 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_4360.add_16bit_int(2) # PASSWORD
                block_4360.add_16bit_int(2) # PASSWORD
                EM24_slave_ctx.setValues(3, 4360, block_4360.to_registers())
                EM24_slave_ctx.setValues(4, 4360, block_4360.to_registers())

                block_40960 = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Little)
                block_40960.add_16bit_int(1) # Type of application
                block_40960.add_16bit_int(3) # Default page for selector position “LOCK”
                block_40960.add_16bit_int(1) # Default page for selector position “1”
                block_40960.add_16bit_int(3) # Default page for selector position “2”
                block_40960.add_16bit_int(3) # Default page for selector position “kvarh”
                block_40960.add_16bit_int(1) # ID code of user 1
                block_40960.add_16bit_int(2) # ID code of user 2
                block_40960.add_16bit_int(3) # ID code of user 3
                EM24_slave_ctx.setValues(3, 40960, block_40960.to_registers())
                EM24_slave_ctx.setValues(4, 40960, block_40960.to_registers())

                update_t_stop = threading.Event()
                update_t = threading.Thread(
                    target=t_update,
                    name=f"t_update_{address}",
                    args=(
                        EM24_slave_ctx,
                        SE7K_slave_ctx,
                        update_t_stop,
                        meter_module,
                        meter_device,
                        confparser[meter].getfloat("refresh_rate", fallback=default_config["meters"]["refresh_rate"]),
                        confparser[meter].getfloat("full_refresh_rate", fallback=default_config["meters"]["full_refresh_rate"]),
                        confparser[meter].getfloat("power_filter_tau", fallback=default_config["meters"]["power_filter_tau"])
                    )
                )

                threads.append(update_t)
                thread_stops.append(update_t_stop)

                slaves.update({1: EM24_slave_ctx})
                slaves.update({2: SE7K_slave_ctx})
                logger.info(f"Created {update_t}: {meter} {meter_type} {meter_device}")

        if not slaves:
            logger.warning(f"No meters defined in {args.config}")

        config_framer = confparser["server"].get("framer", fallback=default_config["server"]["framer"])
        framer = False

        if config_framer == "socket":
            framer = ModbusSocketFramer
        elif config_framer == "rtu":
            framer = ModbusRtuFramer

        identity = ModbusDeviceIdentification()
        server_ctx = ModbusServerContext(slaves=slaves, single=False)

        time.sleep(1)
    
        for t in threads:
            t.start()
            logger.info(f"Starting {t}")

        StartMyTcpServer(
            server_ctx,
            framer=framer,
            identity=identity,
            address=(
                confparser["server"].get("address", fallback=default_config["server"]["address"]),
                confparser["server"].getint("port", fallback=default_config["server"]["port"])
            )
        )
    except KeyboardInterrupt:
        pass
    finally:
        for t_stop in thread_stops:
            t_stop.set()
        for t in threads:
            t.join()

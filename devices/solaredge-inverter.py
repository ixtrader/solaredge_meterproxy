import logging
import re
import solaredge_modbus


def device(config):

    # Configuration parameters:
    #
    # timeout   seconds to wait for a response, default: 1
    # retries   number of retries, default: 3
    # unit      modbus address, default: 1
    #
    # For Modbus TCP:
    # host      ip or hostname
    # port      modbus tcp port
    #
    # For Modbus RTU:
    # device    serial device, e.g. /dev/ttyUSB0
    # stopbits  number of stop bits
    # parity    parity setting, N, E or O
    # baud      baud rate

    timeout = config.getfloat("timeout", fallback=1)
    retries = config.getint("retries", fallback=3)
    unit = config.getint("src_address", fallback=1)

    host = config.get("host", fallback=False)
    port = config.getint("port", fallback=False)
    device = config.get("device", fallback=False)

    if device:
        stopbits = config.getint("stopbits", fallback=1)
        parity = config.get("parity", fallback="N")
        baud = config.getint("baud", fallback=2400)

        if (parity
                and parity.upper() in ["N", "E", "O"]):
            parity = parity.upper()
        else:
            parity = False

        return solaredge_modbus.Inverter(
            device=device,
            stopbits=stopbits,
            parity=parity,
            baud=baud,
            timeout=timeout,
            retries=retries,
            unit=unit
        )
    else:
        return solaredge_modbus.Inverter(
            host=host,
            port=port,
            timeout=timeout,
            retries=retries,
            unit=unit
        )


def _discover(device, attribute, probe):
    """Sondiert angeschlossene Geraete einmalig und merkt sich das Ergebnis.

    Ohne Zwischenspeicherung wird die DID jedes moeglichen Zaehlers und jeder
    moeglichen Batterie in jedem Lesezyklus erneut abgefragt; nicht vorhandene
    Geraete laufen dabei in den Modbus-Timeout und verzoegern die Messwerte.
    """
    cached = getattr(device, attribute, None)
    if not cached:
        cached = probe()
        if cached:
            setattr(device, attribute, cached)
    return cached


def _typed_keys(raw):
    return {f"{key}_{type(value).__name__}": value for key, value in raw.items()}


def power_values(device, meter="Meter1"):
    """Liest nur die Wirkleistungsregister des angeschlossenen Zaehlers.

    Die Register liegen zusammenhaengend (0x9D0E..0x9D12) und werden in einer
    einzigen Modbus-Transaktion gelesen. Das haelt die Totzeit der
    Victron-Nullregelung klein, die ausschliesslich die Wirkleistung auswertet.
    :param device: Initialisiertes Inverter-Objekt.
    :param meter: Name des Zaehlers, wie ihn `solaredge_modbus` vergibt.
    :return: Dictionary mit den Rohwerten, Schluessel wie in :func:`values`.
    """
    params = _discover(device, "_proxy_meters", device.meters).get(meter)
    if not params:
        return {}

    keys = ("power", "l1_power", "l2_power", "l3_power", "power_scale")
    registers = {k: v for k, v in params.registers.items() if k in keys}

    return _typed_keys(params._read_all(registers, solaredge_modbus.registerType.HOLDING))


def values(device):
    if not device:
        return {}

    logger = logging.getLogger()
    logger.debug(f"device: {device}")

    values = {}
    inverter_values=device.read_all()
    
    # append type to key to prevent key name collision with legacy values
    values = {key+'_'+re.search('\'(.*)\'',str(type(value))).group(1):value for key, value in inverter_values.items()}  
    
    meters = _discover(device, "_proxy_meters", device.meters)
    batteries = _discover(device, "_proxy_batteries", device.batteries)
    values["connected_meters"] = {}
    values["connected_batteries"] = {}

    for meter, params in meters.items():
        meter_values = params.read_all()
        values["connected_meters"][meter] = {key+'_'+re.search('\'(.*)\'',str(type(value))).group(1):value for key, value in meter_values.items()}

    for battery, params in batteries.items():
        battery_values = params.read_all()
        values["connected_batteries"][battery] = {key+'_'+re.search('\'(.*)\'',str(type(value))).group(1):value for key, value in battery_values.items()}

    logger.debug(f"values: {values}")

    # additional values for emulation of SE-WNC-3Y-400-MB-K1 or WattNode WNC-3Y-400-MB

    # TODO Calculate the values for the SE-WNC-3Y-400-MB-K1 meter from the SolarEdge meter provided by SE7K

    meterValues = values["connected_meters"].get("Meter1")
    if not meterValues:
        logger.warning("No Meter1 detected; skipping meter update")
        return {}


    SE_WNC_3Y_400_MB_K1_values = {
        "energy_active": meterValues.get('export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "import_energy_active": meterValues.get('import_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "power_active": meterValues.get('power_int', 0)*10**meterValues.get('power_scale_int', 0),
        "l1_power_active": meterValues.get('l1_power_int', 0)*10**meterValues.get('power_scale_int', 0),
        "l2_power_active": meterValues.get('l2_power_int', 0)*10**meterValues.get('power_scale_int', 0),
        "l3_power_active": meterValues.get('l3_power_int', 0)*10**meterValues.get('power_scale_int', 0),
        "voltage_ln": meterValues.get('voltage_ln_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l1n_voltage": meterValues.get('l1n_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l2n_voltage": meterValues.get('l2n_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l3n_voltage": meterValues.get('l3n_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "voltage_ll": meterValues.get('voltage_ll_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l12_voltage": meterValues.get('l12_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l23_voltage": meterValues.get('l23_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "l31_voltage": meterValues.get('l31_voltage_int', 0)*10**meterValues.get('voltage_scale_int', 0),
        "frequency": meterValues.get('frequency_int', 0)*10**meterValues.get('frequency_scale_int', 0),
        "l1_energy_active": meterValues.get('l1_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l2_energy_active": meterValues.get('l2_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l3_energy_active": meterValues.get('l3_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l1_import_energy_active": meterValues.get('l1_import_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l2_import_energy_active": meterValues.get('l2_import_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l3_import_energy_active": meterValues.get('l3_import_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "export_energy_active": meterValues.get('export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l1_export_energy_active": meterValues.get('l1_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l2_export_energy_active": meterValues.get('l2_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "l3_export_energy_active": meterValues.get('l3_export_energy_active_int', 0)*10**meterValues.get('energy_active_scale_int', 0),
        "energy_reactive": 0.0,
        "l1_energy_reactive": 0.0,
        "l2_energy_reactive": 0.0,
        "l3_energy_reactive": 0.0,
        "energy_apparent": meterValues.get('import_energy_apparent_int', 0)*10**meterValues.get('energy_apparent_scale_int', 0),
        "l1_energy_apparent": meterValues.get('l1_import_energy_apparent_int', 0)*10**meterValues.get('energy_apparent_scale_int', 0),
        "l2_energy_apparent": meterValues.get('l2_import_energy_apparent_int', 0)*10**meterValues.get('energy_apparent_scale_int', 0),
        "l3_energy_apparent": meterValues.get('l3_import_energy_apparent_int', 0)*10**meterValues.get('energy_apparent_scale_int', 0),
        "power_factor": meterValues.get('power_factor_int', 0)*10**meterValues.get('power_factor_scale_int', 0),
        "l1_power_factor": meterValues.get('l1_power_factor_int', 0)*10**meterValues.get('power_factor_scale_int', 0),
        "l2_power_factor": meterValues.get('l2_power_factor_int', 0)*10**meterValues.get('power_factor_scale_int', 0),
        "l3_power_factor": meterValues.get('l3_power_factor_int', 0)*10**meterValues.get('power_factor_scale_int', 0),
        "power_reactive": meterValues.get('power_reactive_int', 0)*10**meterValues.get('power_reactive_scale_int', 0),
        "l1_power_reactive": meterValues.get('l1_power_reactive_int', 0)*10**meterValues.get('power_reactive_scale_int', 0),
        "l2_power_reactive": meterValues.get('l2_power_reactive_int', 0)*10**meterValues.get('power_reactive_scale_int', 0),
        "l3_power_reactive": meterValues.get('l3_power_reactive_int', 0)*10**meterValues.get('power_reactive_scale_int', 0),
        "power_apparent": meterValues.get('power_apparent_int', 0)*10**meterValues.get('power_apparent_scale_int', 0),
        "l1_power_apparent": meterValues.get('l1_power_apparent_int', 0)*10**meterValues.get('power_apparent_scale_int', 0),
        "l2_power_apparent": meterValues.get('l2_power_apparent_int', 0)*10**meterValues.get('power_apparent_scale_int', 0),
        "l3_power_apparent": meterValues.get('l3_power_apparent_int', 0)*10**meterValues.get('power_apparent_scale_int', 0),
        "l1_current": meterValues.get('l1_current_int', 0)*10**meterValues.get('current_scale_int', 0),
        "l2_current": meterValues.get('l2_current_int', 0)*10**meterValues.get('current_scale_int', 0),
        "l3_current": meterValues.get('l3_current_int', 0)*10**meterValues.get('current_scale_int', 0),
        "demand_power_active": 0.0,
        "minimum_demand_power_active": 0.0,
        "maximum_demand_power_active": 0.0,
        "demand_power_apparent": 0.0,
        "l1_demand_power_active": 0.0,
        "l2_demand_power_active": 0.0,
        "l3_demand_power_active": 0.0,
    }


    return dict(list(values.items()) + list(SE_WNC_3Y_400_MB_K1_values.items()))
    # append type to key
**Project Description and Comments:**

This project facilitates the transmission of data from the Solaredge inverter, specifically from the connected Modbus SE-MTR-3Y-400V-A, acting as an EM24 Garvazzi meter, to other devices. In my setup, these include a Victron MultiPlus II GX and an OpenWB Wallbox. It's also possible to have additional consumers capable of reading data from the EM24.

**Important Commands:**

*Manually starting the service:*
```bash
sudo /usr/bin/python3 /home/martin/gitHubClones/solaredge_meterproxy/SE7K-EM24-proxy-tcp.py -c /home/martin/gitHubClones/solaredge_meterproxy/SE-MTR-3Y-400V-A.conf
```

*If set up as a service:*
```bash
sudo systemctl status modbus-proxy.service
sudo systemctl stop modbus-proxy.service
sudo systemctl start modbus-proxy.service
sudo journalctl -u modbus-proxy.service      # view log
```

*Installation on a Raspberry Pi:*
```bash
sudo pip3 install 'pymodbus<3.0.0'
sudo pip3 install 'solaredge_modbus<0.7.1'
cd ~
mkdir gitHubClones
cd gitHubClones/  
git clone git@github.com:ixtrader/solaredge_modbus.git   # [if not necessary]
git clone git@github.com:ixtrader/solaredge_meterproxy.git
cd solaredge_meterproxy
git checkout -b ixtrader-master-new origin/ixtrader-master-new   # as of February 2024, this is the functional version
# the only manual step is now to update the adress of the solaredge inverter in the file /home/martin/gitHubClones/solaredge_meterproxy/SE-MTR-3Y-400V-A.conf
# in line 27 "host=solaredge.fritz.box" with your inverter address
```

**Service Installation as Root:**
```bash
vi /etc/systemd/system/modbus-proxy.service   # see contents of the modbus-proxy.service file
sudo systemctl daemon-reload
sudo systemctl enable modbus-proxy.service
```

**Contents of the modbus-proxy.service file:**
```ini
[Unit]
Description=Modbus Proxy

[Service]
ExecStart=/usr/bin/python3 /home/martin/gitHubClones/solaredge_meterproxy/SE7K-EM24-proxy-tcp.py -c /home/martin/gitHubClones/solaredge_meterproxy/SE-MTR-3Y-400V-A.conf
Restart=always
User=root
Group=root
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
```

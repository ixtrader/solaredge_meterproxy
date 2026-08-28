FROM python:3.11-slim

WORKDIR /app

# solaredge_modbus 0.8.0 uses the Pymodbus 3.x API.
RUN pip install --no-cache-dir \
    'pymodbus>=3.5.0,<3.6.0' \
    'solaredge-modbus==0.8.0' \
    influxdb>=5.3.0 \
    requests>=2.23.0 \
    sdm_modbus>=0.5.0

COPY devices/ ./devices/
COPY SE7K-EM24-proxy-tcp.py .

# Overridden per deployment via the compose file / TrueNAS app config.
ENV CONFIG_FILE=/config/SE-MTR-3Y-400V-A.conf

EXPOSE 502

ENTRYPOINT ["python3", "SE7K-EM24-proxy-tcp.py"]
CMD ["-c", "/config/SE-MTR-3Y-400V-A.conf"]

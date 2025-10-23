* python3.14 -m pip install uv
* python3.14 -m uv sync
* source /.venv/bin/activate
* west init --local application
* west update
* python3.14 -m west packages pip --install
* python3.14 -m west sdk install
* python3.14 -m west build  --board nrf54l15dk/nrf54l15/cpuapp --pristine=always application

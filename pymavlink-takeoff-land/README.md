# Pymavlink Takeoff and Land
## Resources
- https://mavlink.io/en/mavgen_python/
- https://www.ardusub.com/developers/pymavlink.html

## Setup (Ubuntu)

Create a virtual environment
```shell
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

Install venv if the above command fails
```shell
sudo apt update
sudo apt install python3-venv
```

Make sure all the following commands are run after activating the virtual environment.

Install dependencies
```shell
pip install -r requirements.txt
```

Run the code
```shell
python3 main.py [device_connection_string]

# usually
python3 main.py /dev/ttyACM0
```

## Setup (Windows)
Create a virtual environment
```powershell
python -m venv .venv
.venv/Scripts/Activate.ps1
```

Make sure all the following commands are run after activating the virtual environment.

Install dependencies
```powershell
pip install -r requirements.txt
```

Run the code
```powershell
python main.py [device_connection_string]

# eg
python main.py COM5
```
Check `Device manager` on Windows to check which COM port the pixhawk is connected to

## Run using docker
```shell
docker build --no-cache -t mavlink-py-image .
docker run -it --rm -v ./:/app --device=/dev/ttyACM0 mavlink-py-image
```

```shell
python3 main.py [device_connection_string]

# usually
python3 main.py /dev/ttyACM0
```

# FastAPI Websockets Webserver
## Resources
- https://fastapi.tiangolo.com/tutorial/
- https://fastapi.tiangolo.com/#installation

## Setup (Ubuntu)
Create and activate virtual environment
```shell
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

If the above command fails, install `venv`
```shell
sudo apt update
sudo apt install python3-venv
```

Make sure to activate the virtual environment before running the following commands.

Install dependencies
```shell
pip install -r requirements.txt
```

Run the program
```shell
python3 main.py
```

## Setup (Windows)
Create and activate virtual environment
```powershell
python -m venv .venv --system-site-packages
.venv/Scripts/Activate.ps1
```

Install dependencies
```powershell
pip install -r requirements.txt
```

Run the program
```powershell
python main.py
```

## Run in docker container
```shell
docker build --no-cache -t server-python .
docker run --rm -it -v ./:/app fastapi-server
```

```shell
python3 main.py
```

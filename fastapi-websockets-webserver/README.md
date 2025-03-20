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
fastapi dev main.py --host 0.0.0.0 --port 8000
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
fastapi main:app --host 0.0.0.0 --port 8000
```

## Run in docker container
```shell
docker build --no-cache -t server-python .
docker run --rm -it -p 8000:8000 -v ./:/app fastapi-server
```

```shell
uvicorn main:app --host 0.0.0.0 --port 8000
```

## Running tests
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
pip install -r dev_requirements.txt
```

Run tests using `pytest`
```shell
python3 -m pytest
```

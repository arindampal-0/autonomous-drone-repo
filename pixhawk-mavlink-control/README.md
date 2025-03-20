# Pixhawk mavlink control

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
python3 main.py
```

## Run using docker
```shell
docker build --no-cache -t mavlink-py-image .
docker run -it --rm -v ./:/app mavlink-py-image
```

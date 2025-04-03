# Mavlink Connection

## Setup (Ubuntu)

## Scratch Setup (Ubuntu)
Install python virtual environment if not done already
```shell
sudo apt update
sudo apt install python3-venv
```


Create and activate virtual environment
```shell
python3 -m venv .venv --system-site-packages --symlinks
source .venv/bin/activate
```

Create a ROS2 workspace
```shell
mkdir src
colcon build
```

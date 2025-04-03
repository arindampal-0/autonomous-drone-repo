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

Make sure to activate the virtual environment before running the following command

Make sure that ROS2 is installed and setup before running the following commands

Create a ROS2 workspace
```shell
mkdir src
colcon build
```

Create a simple package
```shell
ros2 pkg create --build-type ament_python --dependencies rclpy --destination-directory src mavlink_connection
```

`src/mavlink_connection/mavlink_connection/mavlink_connection.py`
```python
#!/usr/bin/env python3
"""mavlink_connection file"""

import sys

import rclpy
from rclpy.node import Node

class MavlinkConnection(Node):
    """MavlinkConnection Node"""
    def __init__(self):
        super().__init__("mavlink_connection")
        self.get_logger().info("Mavlink Connection Node")


def main(args=None):
    """main function"""
    rclpy.init(args=args)

    mavlink_connection_node = MavlinkConnection()
    rclpy.spin(mavlink_connection_node)

    mavlink_connection_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)

```

`src/mavlink_connection/setup.cfg`
```yaml
[build_scripts]
executable=/usr/bin/env python3
...
```

`src/mavlink_connection/setup.py`
```python
from setuptools import find_packages, setup

package_name = 'mavlink_connection'

setup(
    ...
    entry_points={
        'console_scripts': [
            "mavlink_connection = mavlink_connection.mavlink_connection:main"
        ],
    },
)

```

Build and run the node
```shell
colcon build --packages-select mavlink_connection
source install/setup.bash
ros2 run mavlink_connection mavlink_connection
```

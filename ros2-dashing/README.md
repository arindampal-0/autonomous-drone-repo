# ROS2 Dashing Workspace with ZED SDK wrapper
## Docker command
First pull the ROS2 Dashing docker image
```shell
docker pull ros:dashing-ros-base
```

Run the container
```shell
docker run -it --rm -v $(pwd):/ros2-dashing ros:dashing-ros-base
```
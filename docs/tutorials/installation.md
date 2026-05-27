# Installation

HumaWare targets ROS 2 Jazzy as the baseline.

## System Setup

Install ROS 2 Jazzy and colcon for your platform, then clone the repository into a workspace.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Docker

Use `containers/Dockerfile.jazzy` for a repeatable development image.

```bash
docker build -f containers/Dockerfile.jazzy -t humaware:jazzy .
docker run --rm -it --net=host -v "$PWD":/workspaces/humaware humaware:jazzy
```

# cybergear_m5_ros2

ROS2 package for Xiaomi Cybergear.

> [!WARNING]  
> Xiaomi Cybergear can generate high torque, so please make control parameter adjustments at your own risk.

## Dev Environments

* Host OS
  * ubuntu 20.04

* Applciations
  * Docker       : docker-ce 24.0.6
  * Container OS : Ubuntu 22.04
  * Middleware   : ROS2 (humble)

## How to build dev-environments

Create dev-environment on docker using follow commands.
`docker compose up` command launch terminator (blue color) that develop with ros2 humble.
This docker container contains ros2-humble and necessary software to launch samples.

```bash
cd docker

# please check and modify follow env vairables at this script.
# WORKSPACE_DIR ... mount diretory
# SERIAL_DEVICE ... serial device name
bash generate_env.bash

# build docker container
docker compose build

# launch container
docker compose up
```

## Build from source

```bash
source /opt/ros/humble/setup.bash

# make ros2 workspace
export COLCON_WS=~/ws
mkdir -p $COLCON_WS/src

# clone cybergear_m5 source
cd $COLCON_WS/src
git clone git@github.com:project-sternbergia/cybergear_m5_ros2.git
cd ../

# build ros2 package
colcon build
```

## Prepare M5 stack

Before launch sample, please write `cybergear_m5_bridge.ino` to m5 stack.
Please refer [cyberger_m5](https://github.com/project-sternbergia/cybergear_m5) repository.

## How to run sample bridge node

### rviz2 samples (joint_state_publisher_gui samples)

Please test simple sample code.

```bash
cd $COLCON_WS
source install/setup.bash

# 1dof cybergear position control sample
# you control cybergear via joint_state_publisher_gui
# cybergear can id : 0x7F
ros2 launch cybergear_m5_bringup 1dof_position_sample.launch.xml

# 2dof cybergear position control sample
# you control cybergear via joint_state_publisher_gui
# cybergear_1 can id : 0x7F
# cybergear_2 can id : 0x7E
ros2 launch cybergear_m5_bringup 2dof_position_sample.launch.xml
```

If you want to change control parameter, please check config file at `cybergear_m5_description/config` directory.

`2dof_position_sample.launch.xml`

![cybergear_ros2_bridge_example](docs/img/cybergear_ros2_bridge_sample.gif)

## LICENSE

* MIT

## References

* [cybergear_m5](https://github.com/project-sternbergia/cybergear_m5)

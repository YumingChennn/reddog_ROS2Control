# Reddog ROS 2 Control

This repository provides ROS 2 control integration for the **Reddog** quadruped robot. It integrates simulation and hardware support based on several existing open-source projects.

## References

### Unitree Mujoco Hardware Interface
- **Source**: [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)  
- **Path**: `hardwares/hardware_unitree_mujoco`

### XSens MTi Driver for ROS 2
- **Source**: [DEMCON/ros2_xsens_mti_driver](https://github.com/DEMCON/ros2_xsens_mti_driver)  
- **Path**: `libraries/ros2_xsens_mti_driver`

### XSens MTi Driver for ROS 2
- **Source**: xsens mti's website

### Reddog ROS 2 Control Implementation
- **Source**: [luoluoluoouo/reddog_ROS2Control](https://github.com/luoluoluoouo/reddog_ROS2Control)

## Quick start
```
git clone git@github.com:YumingChennn/reddog_ROS2Control.git
```
```
open descriptions/reddog_description/config/robot_control.yaml
```
Change the model path in the robot_control.yaml
```
policy_path: /your/absolute/path/to/reddog_description/config/legged_gym/policy_him2.pt
config_path: /your/absolute/path/to/reddog_description/config/legged_gym/reddog_him.yaml
```

And build/source/launch
```
cd ~/ros2_control
colcon build
source install/setup.bash
```

```
source src/setup/setup_ports.sh
ros2 launch rl_quadruped_controller bringup.launch.py pkg_description:=reddog_description
```

Hardware Manager

To build and run the CAN node for motor and IMU communication:

```
cd ~/reddog_ws/hardware_manager/build
cmake ..
make
sudo ./can_node_motor_imu
```

Change the mode of controller to move
```
ros2 topic pub /mode std_msgs/msg/String "data: 'sit'"
ros2 topic pub /mode std_msgs/msg/String "data: 'stand'"
ros2 topic pub /mode std_msgs/msg/String "data: 'move'"
```
> Please set to 'stand' mode before you set to 'move' mode

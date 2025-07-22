# Reddog ROS2 control
hardwares/hardware_unitree_mujoco ref: [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control/tree/main/hardwares/hardware_unitree_mujoco)
libraries/ros2_xsens_mti_driver ref: [DEMCON/ros2_xsens_mti_driver](https://github.com/DEMCON/ros2_xsens_mti_driver)


## Quick start
```
cd ~/ros2_ws/src
git clone https://github.com/luoluoluoouo/reddog_ROS2Control.git
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
cd ~/ros2_ws
colcon build
source install/setup.bash
```

```
source src/setup/setup_ports.sh

ros2 launch hardware_dds_bridge bringup.launch.py 

ros2 launch rl_quadruped_controller bringup.launch.py pkg_description:=reddog_description
```

Change the mode of controller to move
```
ros2 topic pub /mode std_msgs/msg/String "data: 'sit'"
ros2 topic pub /mode std_msgs/msg/String "data: 'stand'"
ros2 topic pub /mode std_msgs/msg/String "data: 'move'"
```
> Please set to 'stand' mode before you set to 'move' mode

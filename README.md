# gazebo_ros_wheel

This is a sample program to control 2 wheeled car with ros_control.
![Image of the gazebo model and rviz](https://pbs.twimg.com/media/CtsIrjPVUAEHcwe.jpg:large)

## Note

I preffer to use libgazebo_ros_imu_sensor.so [Unmerged to gazebo_ros_pkg](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/363)

## Usage
### Launch simulator
    roslaunch rrcar_description empty.launch

### Start controller
- With simulator

    roslaunch rrcar_control rrcar_control.launch

- With real robot

    roslaunch rrcar_control rrcar_control_robot.launch

### Play to control
- With keyboard

    roslaunch rrcar_control teleop_key.launch

- With joystick

    roslaunch rrcar_control teleop_joy.launch

## Hardware
- [VESC - Open Source ESC](http://vedder.se/2015/01/vesc-open-source-esc/)
- Electric wheel motor (e.g. [light small 4 inch electric wheel motor 24V 200W](https://ja.aliexpress.com/store/product/4-inch-single-shaft-hub-motor-24V-100W/1656295_32597304241.html?detailNewVersion=&categoryId=100001709))

## Software
- [ROS](http://ros.org)
 - [ros_control](http://wiki.ros.org/ros_control)
 - [mit-racecar/vesc](https://github.com/mit-racecar/vesc)
- [Gazebo](http://gazebosim.org/)

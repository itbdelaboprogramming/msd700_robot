<!-- 
##########################################
Last Update     : 28 July 2024
Commenter       : Dimas Ridhwana S 



contact person : dimsridhwana@gmail.com
########################################### 
-->


# msd700_control Package

## Description
This package contains every node needed to control the robot. 

##Building package

robot_localization pkg
--------

A robot localization stack developed by ITB de Labo for Nakayama Iron Works's MSD700 product to fuse some localization sensors using [robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package from ROS.

### Launch file
This stack contains several launch files as follows:
* `imu_filter.launch` : this launch file corrects the IMU measurement by accounting for the earth's magnetic field and the robot's tilt [[1]]. This uses the "imu_filter_madgwick" node from the "imu_tools" package and the "hardware_state.py" node.
[1]: http://wiki.ros.org/imu_tools
* `robot_localization.launch` : this launches "imu_filter.launch" and "ekf_localization_node" from "robot_localization" package [[2]].
[2]: https://docs.ros.org/en/noetic/api/robot_localization/html/index.html

### Nodes
This package contains several custom nodes as follows:
* `hardware_state.py` : this node subscribes to the "hardware_state" topic that consists of the `ros_msd700_msgs::HardwareState` type of messages as its main data to process. Please refer to [this link](https://github.com/itbdelaboprogramming/ros_msd700_msgs) for the message details. In the end, this node will publish the "wheel/odom" (`nav_msgs::Odometry`) topic and the "imu/data" (`sensor_msgs::Imu`) topic that will be supplied to the robot_localization node.

* `quat_to_eul.py` : this node can be set to subscribe to the arbitrary topics that contain orientation data in quaternion such as `nav_msgs::Odometry` or `sensor_msgs::Imu`, then this node will convert those quaternions orientation into Euler angles.

### Visualization
This stack provides an R-viz config to visualize both "wheel/odom" and "odometry/filtered" topics by simply running:
```bash
rosrun rviz rviz -d ~/catkin_ws/src/msd_pose_estimation/rviz/pose_estimation.rviz
```

cmd_vel conversion
-------- 


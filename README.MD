# Robot Repository

Description: 
This repo contains all of the robot repository containing robot movement, SLAM, path planning, and simulation

The main task for this robot is creating a map for featureless tunnel autonomously and use the acquired map to localize and move the robot from one point to another

This repo organized in form of ROS1 stack which consist multiple package connecting to one another. 
The packages are listed the following:
- msd700_movement               : Consist all movement functionalities, such as sensor fusion, navigation, and exploration
- msd700_simulation (package)   : Consist files necessary for simulation purposes, the simuation runs in Gazebo using turtlebot env
- msd700_visual (package)       : Consist robot visual for gazebo or rviz use


Here are more detail information for msd700_movement package
- msd700_bringup            : Consist the launch files to execute primitive robot tasks
- msd700_control            : The main task for this package is to merge sensor reading using robot_localization package
- msd700_firmware           : Consist arduino files spesific for ITBdeLabo's msd700 prototype
- msd700_msg                : Consist all of the robot messages
- msd700_navigations        : Contains SLAM localization algorithm, automapping (msd700_explore), and  
- msd700_mapping            : Contains multiple SLAM localization algorithm and the automapping
- third_party               : Other package we're using such as rplidar_ros pkg



# How to Run
### Autonomous Exploration and Mapping

    ```
    roslaunch msd700_navigations msd700_explore.launch 

    ```

### Autonomous Navigation
    ```
    roslaunch msd700_navigations msd700_navigation.launch 

    ```


If you're using robot other than ITBdeLabo's msd700 prototype, add this arguments

| Arguments             | Value         | Description                                                       |
| :---                  | :--           | :---                                                              |  
| open_rviz             | true/false    | Displays robot state and sensor reading on Rviz                   |
| use_simulator         | true/false    | Ignore the robot, use simulator instead (usefull for debugging)   |
| odom_topic            | topic         | Replace with your robot odometry topic, will be merged using EKF  |
| imu_topic             | topic         | Replace with your robot imu topic, will be merged using EKF       | 
|



Use this as reference
```
roslaunch msd700_navigations msd700_navigation.launch 
    open_rviz:=true 
    use_simulator:=true 
    odom_topic:=/wheel/odom 
    imu_topic:=/imu/data
```








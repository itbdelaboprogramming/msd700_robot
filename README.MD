# Robot Repository

Description: 
This repo contains all of the robot repositories containing robot movement, SLAM, path planning, and simulation

The main task for this robot is to create a map for a featureless tunnel autonomously and use the acquired map to localize and move the robot from one point to another

This repo is organized in the form of ROS1 stack, which consists of multiple packages connecting to one another. 
The packages are listed the following:
- msd700_movement                : Consist all movement functionalities, such as sensor fusion, navigation, and exploration
- msd700_simulation (package)    : Consist files necessary for simulation purposes, the simuation runs in Gazebo using turtlebot env
- msd700_visual (package)        : Consist robot visual for gazebo or rviz use


Here are more detail information for msd700_movement package
- msd700_bringup            : Consist the launch files to execute primitive robot tasks
- msd700_control            : The main task for this package is to merge sensor reading using robot_localization package
- msd700_firmware           : Consists Arduino files spesific for ITBdeLabo's msd700 prototype
- msd700_msg                : Consist all of the robot messages
- msd700_navigations        : Contains SLAM localization algorithm, automapping (msd700_explore), and  
- msd700_mapping            : Contains multiple SLAM localization algorithm and the automapping
- third_party               : Other package we're using such as rplidar_ros pkg



# How to Run
1. Always!!! Make sure the robot is ready and settled on the hardware side (Connected to RC and peripherals),
2. Always!!! Make sure you can directly turn off the robot if something unexpected happens
3. Terminology
    - Manual         = Moving the robot using user command (RC or Teleop)
    - Autonomous     = Moving the robot without user command   
    - Teleop         = Tele operating means moving the robot using keyboard on your PC
4. Arguments
    Arguments are the parameters you can use to change some settings in the robot without changing its code
    All robot has all these basic arguments
    | Arguments             | Value         | Description                                                       |
    | :---                  | :--           | :---                                                              |  
    | open_rviz             | true/false    | Displays robot state and sensor reading on Rviz                   |
    | use_simulator         | true/false    | Ignore the robot, use simulator instead (useful for debugging)    |
    | move_forward_only     | true/false    | Disable moving backward                                           |


    To implement those arguments, use this example as reference
    ```
    roslaunch msd700_navigations msd700_navigation.launch open_rviz:=true use_simulator:=true 
    ```


### Teleop (Tele operating) robot 
    ```
    roslaunch msd700_bringup teleop.launch
    ```

### Manual Mapping 
    ```
    roslaunch msd700_navigations msd700_slam.launch open_rviz:=true
    ```

    after finish creating the map, run this command on command prompt
    
    ```
    rosrun map_server map_saver -f ~/catkin_ws/src/msd700_robot/msd700_movement/msd700_navigations/maps/<span style="color:red">{NAME_OF_THE_FILE}</span>.yaml
    ```

### Autonomous Exploration and Mapping

    ```
    roslaunch msd700_navigations msd700_explore.launch 

    ```

### Autonomous Navigation
    ```
    roslaunch msd700_navigations msd700_navigation.launch 

    ```


### Boustrophedon Movement
    ```
    roslaunch msd700_navigations msd700_boustrophedon.launch 

    ```



If you're using robot other than ITBdeLabo's msd700 prototype, add this arguments

| Arguments             | Value         | Description                                                       |
| :---                  | :--           | :---                                                              |  
| open_rviz             | true/false    | Displays robot state and sensor reading on Rviz                   |
| use_simulator         | true/false    | Ignore the robot, use simulator instead (usefull for debugging)   |
| odom_topic            | topic         | Replace with your robot odometry topic, will be merged using EKF  |
| imu_topic             | topic         | Replace with your robot imu topic, will be merged using EKF       | 
|



For detail explanation please refer to this "StepbyStep msd700" documents
https://docs.google.com/document/d/1TlEUj9Pil4I-Mka3oAh8PWbCwtbUBFhPY5Z3MNVvOw4/edit?usp=sharing





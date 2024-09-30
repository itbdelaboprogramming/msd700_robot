# Robot Repository

Description: 
This repo contains all of the robot repositories containing robot movement, SLAM, path planning, and simulation

The main task for this robot is to create a map for a featureless tunnel autonomously and use the acquired map to localize and move the robot from one point to another

This repo is organized in the form of ROS1 stack, which consists of multiple packages connecting to one another. 
The packages are listed the following:
* msd700_movement                : Consist all movement functionalities, such as sensor fusion, navigation, and exploration
* msd700_simulation (package)    : Consist files necessary for simulation purposes, the simuation runs in Gazebo using turtlebot env
* msd700_visual (package)        : Consist robot visual for gazebo or rviz use


Here are more detail information for msd700_movement package
* msd700_bringup            : Consist the launch files to execute primitive robot tasks
* msd700_control            : The main task for this package is to merge sensor reading using robot_localization package
* msd700_firmware           : Consists Arduino files spesific for ITBdeLabo's msd700 prototype
* msd700_msg                : Consist all of the robot messages
* msd700_navigations        : Contains SLAM localization algorithm, automapping (msd700_explore), and  
* msd700_mapping            : Contains multiple SLAM localization algorithm and the automapping
* third_party               : Other package we're using such as rplidar_ros pkg



# How to Run
1. Always!!! Make sure the robot is ready and settled on the hardware side (Connected to RC and peripherals),
2. Always!!! Make sure you can directly turn off the robot if something unexpected happens
3. <a name="terminology"></a>Terminology
    * Manual         = Moving the robot using user command (RC or Teleop)
    * Autonomous     = Moving the robot without user command   
    * Teleop         = Tele operating means moving the robot using keyboard on your PC
    * Converge       = Converge is the condition when lidar scan reading (green dots) are aligned with the loaded map 
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
   Use this to launch the Teleop
   ```
   roslaunch msd700_bringup teleop.launch
   ```

### Manual Mapping 
   Use this to launch the manual mapping
   ```
   roslaunch msd700_navigations msd700_slam.launch open_rviz:=true
   ```

after finish creating the map, run this command on the command prompt
   ```
   // Change the {NAME_OF_FILE} as you wanted
   rosrun map_server map_saver -f ~/catkin_ws/src/msd700_robot/msd700_movement/msd700_navigations/maps/{NAME_OF_THE_FILE}.yaml
   ```

### Autonomous Exploration and Mapping
   Use this to launch the autonomous mapping
   ```
   roslaunch msd700_navigations msd700_explore.launch 
   ```

 after finish creating the map, run this command on the command prompt
   ```
   // Change the {NAME_OF_FILE} as you wanted
   rosrun map_server map_saver -f ~/catkin_ws/src/msd700_robot/msd700_movement/msd700_navigations/maps/{NAME_OF_THE_FILE}.yaml
   ```

### Autonomous Navigation
   Use this to launch the autonomous navigation
   ```
   roslaunch msd700_navigations msd700_navigation.launch 
   ```
    
 Rviz will open with a robot and a map (grey color with black representing walls), then follow this step,
   * Put the estimated location in the map using 2D Pose Estimate
   * Rotate the robot 360 degree [manually](#terminology) until the map in Rviz [converge](#terminology)
     PICTURESSSSS
   * Navigate the goals using 2D Nav Goal by clicking and then dragging the arrow
     PICTURESSSSS
   * The robot will automatically create a path and move to the goal autonomously while avoiding obstacles


### Boustrophedon Movement
   ```
   roslaunch msd700_navigations msd700_boustrophedon.launch 
   ```

 Rviz will open, then follow this step,
   * Click Publish Points on top of the Rviz window
   * Give a closed environment where the robot needs to operate
      PICTURESSSSS
   * The robot will automatically create a path to do boustrophedon movement 
 

If you're using robot other than ITBdeLabo's msd700 prototype, add this arguments

| Arguments             | Value         | Description                                                       |
| :---                  | :--           | :---                                                              |  
| open_rviz             | true/false    | Displays robot state and sensor reading on Rviz                   |
| use_simulator         | true/false    | Ignore the robot, use simulator instead (usefull for debugging)   |
| odom_topic            | topic         | Replace with your robot odometry topic, will be merged using EKF  |
| imu_topic             | topic         | Replace with your robot imu topic, will be merged using EKF       | 
|



For detailed explanation please refer to this "StepbyStep msd700" documents
https://docs.google.com/document/d/1TlEUj9Pil4I-Mka3oAh8PWbCwtbUBFhPY5Z3MNVvOw4/edit?usp=sharing





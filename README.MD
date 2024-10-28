# MSD700 robot

## Description
This repo contains all of the robot repositories containing robot movement, SLAM, path planning, and simulation. The main task for this robot is to create a map for a **featureless tunnel** autonomously and use the acquired map to localize and move the robot from one point to another. This repo is organized in the form of ROS1 stack, which consists of multiple packages connecting to one another. 

### The packages are listed the following:

| Arguments             | Description            |
| :---                  | :--           | 
| msd700_movement       | This is the main part of the stack which consists of all movement functionalities, such as sensor fusion, navigation, exploration, and coverage movement. |
| msd700_simulation       | Consisting of files necessary for simulation purposes, the simulation runs in Gazebo using a turtlebot environment. This package stored files that define the simulator world however we will not directly use any files from this package to perform robot simulation. |
| msd700_visual        | Consist robot visuals for gazebo or rviz. Same as msd700_simulation, this package stored files for defining robot visuals. |
|

### msd700_movement package

| Arguments                | Description            |
| :---                     | :--           | 
| msd700_bringup           | Consist the launch files to execute primitive robot tasks |
| msd700_control           | The main task for this package is to merge sensor reading using robot_localization package |
| msd700_firmware          | Consists Arduino files spesific for ITBdeLabo's msd700 prototype |
| msd700_msg               | Consist all of the robot messages |
| msd700_navigations       | Contains SLAM localization algorithm, autonomous mapping (explore) and autonomous navigation |
|


## How to Run
1. Always!!! Make sure the robot is ready and settled on the hardware side (Connected to RC and peripherals),
2. Always!!! Make sure you can directly turn off the robot if something unexpected happens
3. Terminology
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

# Remapping Parameter
If you're using robot other than ITBdeLabo's msd700 prototype, remap this params

| Arguments             | Default         | Description                                                  |
| :---                  | :--             | :---                                                         |  
| odom_topic            | /wheel/odom     | Change according to the topic which your robot publishes     |
| imu_topic             | /imu/data       | Change according to the topic which your robot publishes     | 
| cmd_vel_topic         | /cmd_vel        | Change according to the topic which your robot publishes     | 
|

# Esential Topics
| Topic             | Type          | Description   |
| :---              | :--           | :---          |
| /cmd_vel          | Twist         | Robot commands in form of linear and angular velocity
| /hardware_command | custom        | Contains commands to move the robot
| /odom             | Odom          | Contains robot odometry descriptions
| /imu              | Odom          | Contains robot odometry descriptions



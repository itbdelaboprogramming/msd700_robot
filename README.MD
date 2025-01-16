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
| msd700_msgs              | Consist all of the robot messages |
| msd700_navigations       | Contains SLAM localization algorithm, autonomous mapping (explore) and autonomous navigation |
|



# Robot Repository

Description: 
This repo contains all of the robot repository containing robot movement, SLAM, and path planning

The main task for this robot is creating a map for featureless tunnel autonomously and use the acquired map to localize and move the robot from one point to another

This repo organized in form of ROS1 stack which consist multiple package connecting to one another. 
The packages are listed the following:
    - msd700_launch             : Consist the user-end launch file to execute robot tasks
    - msd700_msg                : Consist all of the robot messages
    - msd700_control            : two main task (convert from cmd_vel to harware_command and integration of robot sensors)
    - msd700_mapping            : Contains multiple SLAM localization algorithm and the automapping
    - msd700_navigation         : Contains navigation 
    - third_party               : such as rplidar_ros pkg




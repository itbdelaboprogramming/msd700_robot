
# msd700_bringup
This package consists some primitive launch files. These launch files might be used in several packages hence assembling them in one package will make it easier. There are several launch files consist in this package, such as


| Launch files                | Description            |
| :---                     | :--           | 
| bridger.launch           | Launches the bridger.py that connects the ROS program to robot "prototype" hardware_command |
| lidar_scanner.launch           | Connect the robot with RPLidar A1 and read the lidar sensor data |
| map_server.launch          | launches map_server node that can "save" the new map or "display" the preexisting map |
| multiple_point.launch               | Launches multiple point node to enable pin point multiple target |
| rviz_launch.launch       | Visualize the robot data in RVIZ |
| serial_launch.launch               | Connects the robot to arduino |
| teleop.launch               | Enable moving the robot using keyboard |
|
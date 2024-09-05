# Esential Topics

MSD700_bringup
bridger
- /cmd_vel          | Twist
- /odom             | Odom


MSD700_control
Subscriber (required)

Data                        Type            Author
hardware_state      
- hardware_state    | HardwareState     | custom
- imu/from_filter   | Imu               | bawaan ROS

quat_to_eul
- imu/data_raw      | Imu               | bawaan ROS
- imu/data          | Imu               | bawaan ROS
- odometry/filtered | Odometry          | bawaan ROS
- wheel/odom        | Odometry          | bawaan ROS





Publisher
- 


MSD700_navigation



# Important 
hardware_state as an input
cmd_vel as an output




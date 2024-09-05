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



# Important 
hardware_state as an input
cmd_vel as an output

- urgensi harusnya di sampaikan aja



# TurtleBot
topics
- /imu              | sensor_msgs/Imu           | bawaan ROS
- /joint_states     | sensor_msgs/JointState    | bawaan ROS
- /odom             | nav_msgs/Odometry         | bawaan ROS


/imu              
    orientation
        x
        y
        z
        w
    angular_velocity
        x
        y
        z
    linear_acceleration
        x
        y
        z


/joint_states     
    wheel_right_joint
    wheel_left_joint

/odom             
    pose
    twist


/hardware_state
    int32   right_motor_pulse_delta
    int32   left_motor_pulse_delta  
    float32 heading
    float32 pitch
    float32 roll
    float32 acc_x
    float32 acc_y
    float32 acc_z
    float32 gyr_x
    float32 gyr_y
    float32 gyr_z
    float32 mag_x
    float32 mag_y
    float32 mag_z




odom_pub        = rospy.Publisher('wheel/odom', Odometry, queue_size=1)
imu_raw_pub     = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
mag_pub         = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
imu_pub         = rospy.Publisher('imu/data', Imu, queue_size=1)


















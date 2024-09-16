
# msd700_slam.launch

open_rviz           
use_simulator
slam_methods



    <arg name="open_rviz"       default="false"/>
    <arg name="use_simulator"   default="false"/>
    <arg name="slam_methods"    default="gmapping" doc="slam type [gmapping, cartographer, hector, karto, frontier_exploration]"/>
    <arg name="model"           default="$(find msd700_visual)/urdf/irbot.urdf.xacro"/>
    <arg name="rviz_config"     value="$(find msd700_visual)/rviz/pose_estimation.rviz"/>





msd700_control

yaml diganti




step-by-step
1. roslaunch msd700_simulation msd700_world.launch
2. 
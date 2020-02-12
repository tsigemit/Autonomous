roscore
roslaunch teleop_twist_joy teleop.launch
rosrun my_ex1 wifibot_LowLvCom
rosrun hokuyo_node hokuyo_node
rosrun rviz rviz
rostopic echo /cmd_vel

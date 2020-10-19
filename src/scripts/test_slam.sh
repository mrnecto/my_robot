#!/bin/sh

xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch world_file:=$(rospack find my_robot)/worlds/MyWorld.world" &
sleep 5
xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch my_robot rviz_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"

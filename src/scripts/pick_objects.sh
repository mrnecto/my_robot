#!/bin/sh

xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld.world" &
sleep 5
xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/rtabmap.yaml" &
sleep 5
xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "cd ~/catkin_ws && source devel/setup.bash && roslaunch pick_objects pick_objects.launch"

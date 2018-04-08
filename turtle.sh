#! /bin/bash
source devel/setup.bash
gnome-terminal -e roscore
gnome-terminal -e "rosrun turtlesim turtlesim_node"
gnome-terminal -e "rosrun hw3 hw3test"

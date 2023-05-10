#!/bin/bash

# Source the ROS 2 installation


# Launch ariac_gazebo with trial_name=final in terminal 1
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch ariac_gazebo ariac.launch.py trial_name:=final"

# Launch robot_commander with a 10-second timer in terminal 2
gnome-terminal -- bash -c "source install/setup.bash  && ros2 launch robot_commander robot_commander.launch.py"


# Launch rwa4_group5 in terminal 3
gnome-terminal -- bash -c "source install/setup.bash && sleep 15 && ros2 launch rwa4_group5 rwa4_node.launch.py"


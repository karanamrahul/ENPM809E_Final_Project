# ENPM809E_Final_Project

Packages for assignments and the final project

## Installation

```bash
source /opt/ros/galactic/setup.bash
cd
mkdir -p ~/final_809e_ws/src
cd final_809e_ws/src
git clone https://github.com/karanamrahul/ENPM809E_Final_Project.git
cd ..
sudo apt install python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=final
ros2 launch robot_commander robot_commander.launch.py
ros2 run robot_commander floor_robot_main.py
ros2 launch rwa4_group5 rwa4_node.launch.py

```

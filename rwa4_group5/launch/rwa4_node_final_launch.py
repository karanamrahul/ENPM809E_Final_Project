import launch 
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.substitutions import FindPackageShare
import time

def generate_launch_description():

    
    # Launch ariac.launch.py for gazebo simulation with trial parameter
    ariaclaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("ariac_gazebo"), "/launch", "/ariac.launch.py"]
    ), launch_arguments={'trial': 'final'}.items())
    
    # Launch
    robotcommander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robot_commander"), "/launch", "/robot_commander.launch.py"]
        )   
    )
    
    delay = TimerAction(period=20.0, actions=[IncludeLaunchDescription( PythonLaunchDescriptionSource([FindPackageShare("rwa4_group5"), "/launch", "/rwa4_node.launch.py"]))])

   # Launch rwa4_node.launch.py
    rwa4_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("rwa4_group5"), "/launch", "/rwa4_node.launch.py"]))
    
  
   
    return launch.LaunchDescription([ariaclaunch, robotcommander, delay, rwa4_launch])
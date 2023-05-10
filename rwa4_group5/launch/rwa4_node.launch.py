from launch import LaunchDescription
from launch_ros.actions import Node
import os
import rclpy
import sys
# this function is needed
  
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    parameter_file = os.path.join(
    get_package_share_directory('rwa4_group5'),
    'config',
    'order.yaml')
    ld = LaunchDescription() # instantiate a Launchdescription object
    publisher_node = Node( # declare your Node
    package="rwa4_group5", # package name
    executable="competition_interface",
    output = 'screen',
    parameters=[parameter_file],
    # executable as set in setup.py
    )
    ld.add_action(publisher_node) # add each Node to the LaunchDescription object
    return ld # return the LaunchDescription object
  
  


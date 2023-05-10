#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data 
from std_srvs.srv import Trigger
from rclpy.time import Duration
from competitor_interfaces.msg import Robots as RobotsMsg
from competitor_interfaces.srv import (
    EnterToolChanger , ExitToolChanger,PlacePartInTray,PickupTray,
    MoveTrayToAGV,PlaceTrayOnAGV,RetractFromAGV,PickupPart,
    MovePartToAGV)
from ariac_msgs.msg import Part, VacuumGripperState, CompetitionState, Order, AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
from ariac_msgs.srv import MoveAGV, ChangeGripper, VacuumGripperControl
from tf_transformations import quaternion_from_euler
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import PyKDL


class RobotCommanderInterface(Node):
    '''
    Class for a robot commander node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.order_group = MutuallyExclusiveCallbackGroup()

        # Flag to indicate if the kit has been completed
        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None
        
        self._gripper_states = {
                True: 'enabled',
                False: 'disabled'
            }
        
        self._gripper_type_dict = {
                1: 'Parts Gripper',
                2: 'Tray Gripper'
        }
        
        self._orders_dict = {}  # List to store the received orders
        self._table_dict = {}  # Dictionary to store the parts on each table
        self._part_list = [] # List to store the parts
        
        self._table_needed = [] # List to store the table/tray needed 
        self._parts_needed = [] # List to store the parts needed
                
        # Flags for first messages 
        self._table1_tray_first_msg = False
        self._table2_tray_first_msg = False
        
        self._left_bin_first_msg = False
        self._right_bin_first_msg = False
        
        self._num_orders_received = 0
        self._orders_received = False
        
        self.agv_number = None
        
        # Dictionaries for colors and part types
        self._colors = {"0": "Red", "1": "Green", "2": "Blue", "3": "Orange", "4": "Purple"}
        self._part_types = {"10": "Battery", "11":"Pump", "12": "Sensor", "13":"Regulator"}
        
        parameter_file = os.path.join(
            get_package_share_directory('rwa4_group5'),
            'config',
            'order.yaml')
        self.declare_parameter('order_id', '')
        self.get_logger().info(f"Parameter file: {parameter_file}")
        self.get_logger().info(f"Declared parameter 'order_id' with default value: {self.get_parameter('order_id').value}")
        self._order_id = self.get_parameter('order_id').value
        if not self._order_id:
            self._order_id = self.read_yaml(parameter_file)['competition_interface']['ros__parameters']['order_id']
            self._order_id
           
        self._order_id = int(self._order_id)
        print("[WARN]",self._order_id)

        
        # Subscriber for the "/ariac/orders" topic.
        self.subscription_orders = self.create_subscription(
            Order,
            '/ariac/orders',
            self._order_callback,
            10,
            callback_group=self.order_group)
        
        # Subscriber for the "/ariac/sensors/table1_camera/image" topic.
        self.table1_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/table1_camera/image',
            self._table_camera_1_callback,
            qos_profile_sensor_data,
            callback_group=self.order_group
        )
        
        # Subscriber for the "/ariac/sensors/table2_camera/image" topic.
        self.table2_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/table2_camera/image',
            self._table_camera_2_callback,
            qos_profile_sensor_data,
            callback_group=self.order_group
        )
        
        # Subscriber for the "/ariac/sensors/left_bins_camera/image" topic.
        self.left_bin_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/left_bins_camera/image',
            self._left_bin_camera_callback,
            qos_profile_sensor_data
        )
        
        # Subscriber for the "/ariac/sensors/right_bins_camera/image" topic.
        self.right_bin_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/right_bins_camera/image',
            self._right_bin_camera_callback,
            qos_profile_sensor_data
        )
        
        # subscriber
        self.create_subscription(
            CompetitionState, 
            '/ariac/competition_state',
            self._competition_state_cb, 1)
        
        # Subscriber to the floor gripper state topic
        self._floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            '/ariac/floor_robot_gripper_state',
            self._floor_robot_gripper_state_cb,
            qos_profile_sensor_data)

        # Attribute to store the current state of the floor robot gripper
        self._floor_robot_gripper_state = VacuumGripperState()

        # timer
        self._robot_action_timer = self.create_timer(
            1, 
            self._robot_action_timer_callback,
            callback_group=self.timer_group)

        # Service client for starting the competition
        self._start_competition_client = self.create_client(
            Trigger, 
            '/ariac/start_competition')

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home_client = self.create_client(
            Trigger, 
            '/competitor/floor_robot/go_home',
            callback_group=self.service_group)
        
        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, 
            '/competitor/floor_robot/enter_tool_changer',
            callback_group=self.service_group)
        
        # Service client for entering the gripper slot
        self.retract_from_tool_changer_client = self.create_client(
            ExitToolChanger, 
            '/competitor/floor_robot/exit_tool_changer',
            callback_group=self.service_group)
        
        # Service Client for placing the part in the tray
        self.place_part_in_tray_client = self.create_client(
            PlacePartInTray,
            '/competitor/floor_robot/place_part_in_tray',
            callback_group=self.service_group)

        # Service Client for picking up the tray
        self.pickup_tray_client = self.create_client(
            PickupTray,
            '/competitor/floor_robot/pickup_tray',
            callback_group=self.service_group)

        # Service Client for moving the tray to agv
        self.move_tray_to_agv_client=self.create_client(
            MoveTrayToAGV,
            '/competitor/floor_robot/move_tray_to_agv',
            callback_group=self.service_group)

        # Service Client for placing the tray on agv
        self.place_tray_on_agv_client = self.create_client(
            PlaceTrayOnAGV,
            '/competitor/floor_robot/place_tray_on_agv',
            callback_group=self.service_group)

        # Service Client for retracting the tray from agv
        self.retract_from_agv_client = self.create_client(
            RetractFromAGV,
            '/competitor/floor_robot/retract_from_agv',
            callback_group=self.service_group)

        # Service Client for picking up the part
        self.pickup_part_client = self.create_client(
            PickupPart,
            '/competitor/floor_robot/pickup_part',
            callback_group=self.service_group)
   
        # Service Client for moving the part to agv
        self.move_part_to_agv_client = self.create_client(
            MovePartToAGV,
            '/competitor/floor_robot/move_part_to_agv',
            callback_group=self.service_group)
    
        # Service client for turning on/off the vacuum gripper on the floor robot
        self._floor_gripper_enable = self.create_client(
            VacuumGripperControl,
            "/ariac/floor_robot_enable_gripper",
            callback_group=self.service_group)
    
        self._floor_gripper_change = self.create_client(
            ChangeGripper,
            "/ariac/floor_robot_change_gripper",
            callback_group=self.service_group)
    
    ########## hw4 classes ##########
    
    class OrderClass:
        ''' This class represents the KittingOrder class for the RWA-4 assignment
               
                Attributes:
                - id (int): The ID of the order.
                - priority (int): The priority of the order.
                - agv_number (int): The AGV number of the order.
                - tray_id (str): The tray ID of the order.
                - destination (str): The destination of the order.
                - parts (List[Dict[str, str]]): A list of dictionaries to store the parts of the order.

                Methods:
                
                - __str__(self): Overriding the __str__ method to print the order information.
                - __init__(self, order): Constructor for the KittingOrder class.
        
        '''
        
        def __init__(self, order):
            ''' Constructor for the KittingOrder class '''
            
            self.id = order.id
            self.priority = order.priority
            self.agv_number = order.kitting_task.agv_number
            self.tray_id = order.kitting_task.tray_id
            self.destination = order.kitting_task.destination
            self.parts = []
            for part in order.kitting_task.parts: 
                part_info = { # dict for each part
                    'color': part.part.color, 
                    'type': part.part.type,
                    'quadrant': part.quadrant
                }
                self.parts.append(part_info)
                
            

        # Overriding the __str__ method to print the order information
        def __str__(self):
            ''' Overriding the __str__ method to print the order information'''
            return f"Order ID: {self.id}, Priority: {self.priority}, AGV Number: {self.agv_number}, Tray ID: {self.tray_id}, Destination: {self.destination}, Parts: {self.parts}"
    
    class Tray:
        ''' This class represents the Tray class for the RWA-4 assignment
        
            Attributes:
            - id (str): The ID of the tray.
            - pose (Pose): The pose of the tray.
            - sensor_pose (Pose): The pose of the sensor.
        
        
            Methods:
                -   __str__(self): Overriding the __str__ method to print the tray information.
                -   __init__(self, tray_message): Constructor for the Tray class.
                
        
        '''
        def __init__(self, tray_message):
            ''' Constructor for the Tray class'''
            self.id = tray_message.tray_poses[0].id
            self.pose = tray_message.tray_poses[0].pose
            self.sensor_pose = tray_message.sensor_pose
            self.tray_table = None

        def __str__(self):
            ''' Overriding the __str__ method to print the tray information'''
            return f"Tray ID: {self.id}, Pose: {self.pose}, Sensor Pose: {self.sensor_pose}"
    
    class Part:
        ''' This class represents the Part class for the RWA-4 assignment

            Attributes:
                - sensor_pose (Pose): The pose of the sensor.
                - parts (list): A list of the parts detected by the sensor.
                
                
            Methods:

                -   __str__(self): Overriding the __str__ method to print the part information.
    
        '''
        def __init__(self, part_message):
            ''' Constructor for the Part class'''
            self.sensor_pose = part_message.sensor_pose
            self.parts = part_message.part_poses
            self.bin_pos = None
            

        # Overload the __str__ method to check the attributes of the class
        def __str__(self):
            ''' Overriding the __str__ method to print the part information'''
            return f"Sensor Pose: {self.sensor_pose}, Parts: {self.parts}"
    
    def read_yaml(self, path):
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}
    
    ######### hw4 functions #######
            
            
    def _order_callback(self, order):
        ''' Callback function for the "/ariac/orders" topic.

            Args:
                - order (Order): The received order message.
        
        '''
        # Store the order in the list
        self.get_logger().info("Order received ID: " + str(order.id))
        self.order_received = self.OrderClass(order)
        self._orders_dict[str(order.id)] = self.order_received
        self._num_orders_received += 1
        
        if self._num_orders_received == 4:
            self._orders_received = True
        
    def _table_camera_1_callback(self, tray_info):
        
        ''' Callback function for the "/ariac/sensors/table1_camera/image" topic.
        
            Args:
                - tray_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/table1_camera/image" topic.
                
        '''
         
        if not self._table1_tray_first_msg: # Check if the tray is empty
            self.get_logger().info("Table 1 tray info received")
            
            self.table1_tray = self.Tray(tray_info)
            self.table1_tray.tray_table = "kts1"
            
            converted_pose = self._multiply_pose(self.table1_tray.sensor_pose, self.table1_tray.pose) # Convert the pose of the tray to the world frame
            
            self.table1_tray.pose = converted_pose
            
            
            # Add to dictionary
            self._table_dict[str(self.table1_tray.id)] = self.table1_tray 
            self._table1_tray_first_msg = True
        else:
            pass
        
            
        
    def _table_camera_2_callback(self, tray_info):
    
        
        ''' Callback function for the "/ariac/sensors/table2_camera/image" topic.

            Args:
                - tray_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/table2_camera/image" topic.
        
        '''
        
        
        if not self._table2_tray_first_msg and self._orders_received:
            self.get_logger().info("Table 2 tray info received")
            
            self.table2_tray = self.Tray(tray_info)
            self.table2_tray.tray_table = "kts2"
            
            converted_pose = self._multiply_pose(self.table2_tray.sensor_pose, self.table2_tray.pose) # Convert the pose of the tray to the world frame
            
            self.table2_tray.pose = converted_pose
            
            # Add to dictionary
            self._table_dict[str(self.table2_tray.id)] = self.table2_tray
            self._table2_tray_first_msg = True
            
        else:
            pass
        
    def _left_bin_camera_callback(self, part_info):
        ''' Callback function for the "/ariac/sensors/left_bins_camera/image" topic.
        
            Args:
                - part_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/left_bins_camera/image" topic.
            
            '''
            
        if not self._left_bin_first_msg and self._orders_received:
            self.get_logger().info("Left bin info received")
            
            self.left_bin = self.Part(part_info)
            self.left_bin.bin_pos = "left_bins"
            
            for i in range(len(self.left_bin.parts)):
                part_info = {
                    'type': self.left_bin.parts[i].part.type,
                    'color': self.left_bin.parts[i].part.color,
                    'pose': self.left_bin.parts[i].pose,
                    'bin': 'left_bins'
                }
                part_info["pose"] = self._multiply_pose(self.left_bin.sensor_pose, part_info['pose'])

                self._part_list.append(part_info)
                # print(self.part_list)
            
            self._left_bin_first_msg = True
            
        else:
            pass
        
    def _right_bin_camera_callback(self, part_info):
        ''' Callback function for the "/ariac/sensors/right_bins_camera/image" topic. 
        
        
            Args:
                - part_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/right_bins_camera/image" topic.
        '''
                
        if not self._right_bin_first_msg and self._orders_received:
            self.get_logger().info("Right bin info received")
            
            self.right_bin = self.Part(part_info)
            self.right_bin.bin_pos = "right_bins"
            
            for i in range(len(self.right_bin.parts)):
                part_info = {
                    'type': self.right_bin.parts[i].part.type,
                    'color': self.right_bin.parts[i].part.color,
                    'pose': self.right_bin.parts[i].pose,
                    'bin': 'right_bins'
                }
                part_info["pose"] = self._multiply_pose(self.right_bin.sensor_pose, part_info['pose'])

                self._part_list.append(part_info)
                # print(self.part_list)
               
            self._right_bin_first_msg = True
        
        else:
            pass
        
    def _multiply_pose(self, pose1, pose2):
        
        '''
        Use KDL to multiply two poses together.
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose
        
    def _order_logger(self):
        
        
        ''' This function prints the order information,tray pose and the part information.'''
        
        self.get_logger().info("Logging order information")
        self.get_logger().info("Order ID: " + str(self._order_id))
        
        agv_num = self._orders_dict[str(self._order_id)].agv_number
        
        self.agv_number = agv_num
        
        self.get_logger().info('/ariac/move_agv' + str(agv_num))
        
        # Service Client for locking the tray and parts on the agv 
        self.lock_agv_client= self.create_client(
            Trigger,
            '/ariac/agv' + str(agv_num) + '_lock_tray',
            callback_group=self.service_group)
        
        # Service Client for moving the agv to warehosue 
        self.move_agv_client = self.create_client(
            MoveAGV,
            '/ariac/move_agv' + str(agv_num),
            callback_group=self.service_group)
        
        
        tray_id = str(self._orders_dict[str(self._order_id)].tray_id)
        part_ids = self._orders_dict[str(self._order_id)].parts
      
        
        self._table_needed.append(self._table_dict[tray_id])
        
        self.get_logger().info("Tray ID: " + str(self._table_dict[tray_id].id))
        


        taken_parts = []
        
        # print(" Part:")
        for i in range(len(part_ids)):
            part_color = part_ids[i]["color"]
            part_type = part_ids[i]["type"]
            
            self.get_logger().info("Finding Part: " + self._colors[str(part_color)] + " " + self._part_types[str(part_type)])
            
            for j in range(len(self._part_list)):
               
                if self._part_list[j]["color"] == part_color and self._part_list[j]["type"] == part_type and (j not in taken_parts):
                   
                    self._part_list[j]["quadrant"] = part_ids[i]["quadrant"] 
                    
                    
                    self._parts_needed.append(self._part_list[j])
                    taken_parts.append(j)
                    
                    
          
                    break
                
        self.get_logger().info("Completed Saving Order Information")
            
    
    def _competition_state_cb(self, msg: CompetitionState):
        '''
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        '''
        self._competition_state = msg.competition_state

   

    def start_competition(self):
        '''
        Start the competition
        '''
        self.get_logger().info('Waiting for competition state READY')

        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self._competition_started = True
        else:
            self.get_logger().warn('Unable to start competition')

    def goto_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector inside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move inside gripper slot service called')

        request = EnterToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._goto_tool_changer_client.call_async(request)
        
        if gripper_type == "trays":
            gripper_type_req = 2
        elif gripper_type == "parts":
            gripper_type_req = 1
        else:
            raise ValueError('Invalid Gripper Type')

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
                self.change_gripper(gripper_type_req)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')
    
    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._move_floor_robot_home_client.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)
    def retract_from_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector outside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move outside gripper slot service called')

        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self.retract_from_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is retracted from the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract the robot from the tool changer')

   

    def place_part_in_tray(self, robot, agv, quadrant):
        '''
        Places a part in the tray

        Args:
            robot (str): Robot to use
            agv (str): AGV where the tray is located
            quadrant (int): Quadrant oin the tray to place the part in
        '''
        request = PlacePartInTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.quadrant = quadrant

        future = self.place_part_in_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().warn('Deactivate gripper')
                self.set_floor_robot_gripper_state(False)
                self.wait(1)
                self.get_logger().info('Part placed in tray')
            else:
                self.get_logger().warn(f'Service call failed {future.exception()}')

    def pickup_tray(self, robot, tray_id, tray_pose, tray_table):
        """
        Picks up a tray from a table

        Args:
            robot (str): Robot to use
            tray_id (str): ID of the tray to pick up
            tray_pose (Pose): Pose of the tray
            tray_table (str): Name of the table where the tray is located
        """
        self.get_logger().info('Picking Tray service called')
        request = PickupTray.Request()
        
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.tray_pose = tray_pose
        request.tray_station = tray_table

        future = self.pickup_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().warn('Activate gripper')
                self.set_floor_robot_gripper_state(True)
                self.wait(1)
                self.get_logger().warn('Tray picked up')
            else:
                self.get_logger().warn('Failed to pick up tray')
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')
        
        
    def move_tray_to_agv(self, robot, tray_pose, agv):
        '''
        Moves the tray to the AGV

        Args:
            robot (str): Robot to use
            tray_pose (geometry_msgs.msg.Pose): The pose of the tray
            agv (str): The name of the AGV
        '''

        request = MoveTrayToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_pose = tray_pose
        request.agv = agv

        future = self.move_tray_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Tray moved to AGV')
            else:
                self.get_logger().warn('Tray could not be moved to AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
    def place_tray(self, robot,tray_id, agv):
        '''
        Place the tray on the AGV.

        Args:
            robot (str): Name of the robot
            agv (str): Name of the AGV

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Place tray on AGV service called')

        request = PlaceTrayOnAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.tray_id=tray_id

        future = self.place_tray_on_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().warn('Deactivate gripper')
                self.set_floor_robot_gripper_state(False)
                self.get_logger().warn('Tray placed on AGV')
            else:
                self.get_logger().error('Failed to place tray on AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place tray on AGV')

    def retract_from_agv(self, robot, agv):
        '''
        Move the robot to retract from the AGV.

        Args:
            robot (str): Robot to use
            agv (str): AGV to retract from
        '''

        request = RetractFromAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv

        future = self.retract_from_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot has retracted from the AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract the robot from the AGV')
            
            
    def pickup_part(self, robot, part_pose, part_type, part_color, bin_id):
        """
        Picks up a part from a bin using the floor robot.

        Args:
            robot (str): The robot to use.
            part_pose (Pose): The pose of the part.
            part_type (str): The type of the part.
            part_color (str): The color of the part.[]
            bin_id (str): The id of the bin.

        Raises:
            ValueError: If the robot name is invalid.
        """
        self.get_logger().info('Picking Part service called')
        request = PickupPart.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.part_type = part_type
        request.part_color = part_color
        request.bin_side = bin_id

        future = self.pickup_part_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().warn('Activate gripper')
                self.wait(2)
                self.set_floor_robot_gripper_state(True)
                self.get_logger().info('Part picked up successfully')
            else:
                self.get_logger().warn('Failed to pick up part')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick up part')
    from example_interfaces.msg import String


    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        """
        Move a part to the specified AGV and quadrant.

        Args:
            robot (str): Robot name
            part_pose (Pose): Part pose
            agv (str): AGV name
            quadrant (int): Quadrant number in the AGV

        Raises:
            ValueError: If the robot name is invalid
        """
        request = MovePartToAGV.Request()
        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.agv = agv
        request.quadrant = quadrant

        future = self.move_part_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info(f'Part moved to AGV {agv}, quadrant {quadrant}')
            else:
                self.get_logger().warn('Failed to move part to AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the part to the AGV')
            
    def lock_agv(self, agv):
        """
        Lock the tray and parts on the AGV so they do not fall when the AGV is in motion.

        Args:
            agv (str): AGV name
        """

        self.get_logger().info('Lock AGV service called')

        request = Trigger.Request()

        future = self.lock_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'AGV {agv} is locked')
            else:
                self.get_logger().warn(f'Unable to lock AGV {agv}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error(f'Unable to lock AGV {agv}')

    def move_agv_to_warehouse(self, agv):
        """
        Move the AGV to the warehouse.

        Args:
            agv (str): AGV name
        """

        self.get_logger().info(f'Moving AGV {agv} to warehouse')

        request = MoveAGV.Request()
        request.location = 3 # Set the location to the warehouse

        future = self.move_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'AGV {agv} has reached the warehouse')
            else:
                self.get_logger().warn(f'Unable to move AGV {agv} to warehouse')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error(f'Unable to move AGV {agv} to warehouse')
    
    def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        '''Callback for the topic /ariac/floor_robot_gripper_state

        Arguments:
            msg -- VacuumGripperState message
        '''
        self._floor_robot_gripper_state = msg        
    
    def set_floor_robot_gripper_state(self, state):
        '''Set the gripper state of the floor robot.

        Arguments:
            state -- True to enable, False to disable

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f'Gripper is already {self._gripper_states[state]}')
            return

        request = VacuumGripperControl.Request()
        request.enable = state

        future = self._floor_gripper_enable.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().warn(f'Changed gripper state to {self._gripper_states[state]}')
        else:
            self.get_logger().warn('Unable to change gripper state')
            
    def change_gripper(self, type):
        
        request = ChangeGripper.Request()
        request.gripper_type = type
        
        future = self._floor_gripper_change.call_async(request)
        
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error
        
        if future.result().success: 
            self.get_logger().warn(f"Changed the Gripper type to {self._gripper_type_dict[type]}" )
            
        else:
            self.get_logger().warn('Unable to change gripper')
        

        
    def wait(self, duration):
        '''Wait for a specified duration.

        Arguments:
            duration -- Duration to wait in seconds

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error
            
    def _complete_kitting(self):
        self.get_logger().warn('Kitting Started')
        tray_selected = self._table_needed[-1]
        tray_selected_id = tray_selected.id
        tray_selected_pose = tray_selected.pose
        tray_selected_table = tray_selected.tray_table
        agv_selected = "agv" + str(self.agv_number)
        
        # Move robot home
        self.move_robot_home("floor_robot")
        
        # Change gripper type
        self.goto_tool_changer("floor_robot", tray_selected_table, "trays")
        self.retract_from_tool_changer("floor_robot", tray_selected_table, "trays")
        
        # Pick and place tray
        self.pickup_tray("floor_robot", tray_selected_id, tray_selected_pose, tray_selected_table)
        self.move_tray_to_agv("floor_robot", tray_selected_pose, agv_selected)
        self.place_tray("floor_robot", tray_selected_id, agv_selected)
        self.retract_from_agv("floor_robot", agv_selected)
        
        if self._parts_needed[0]["bin"] == "right_bins":
            tray_selected_table = "kts2"
        else:
            tray_selected_table = "kts1"
        
        # Change gripper type
        self.goto_tool_changer("floor_robot", tray_selected_table, "parts")
        self.retract_from_tool_changer("floor_robot", tray_selected_table, "parts")
        
        for part in self._parts_needed:
            # Pick and place parts
            self.pickup_part("floor_robot", part["pose"], part["type"], part["color"], part["bin"])
            self.move_part_to_agv("floor_robot", part["pose"], agv_selected, part["quadrant"])
            self.place_part_in_tray("floor_robot", agv_selected, part["quadrant"])
            self.retract_from_agv("floor_robot", agv_selected)
            
        # move robot home
        self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.lock_agv(str(self.agv_number))
        self.move_agv_to_warehouse(str(self.agv_number))
        
        return
        
        

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''
        
        # complete_order_received = self._table1_tray_first_msg and self._table2_tray_first_msg and self._left_bin_first_msg and self._right_bin_first_msg
        
        if self._competition_state == CompetitionState.READY and not self._competition_started:
                self.start_competition()
                
        if self._kit_completed:
            self.get_logger().info('Kit completed')
            return
        
        self.get_logger().info('Robot action timer callback triggered')
        
        self.wait(3)
        
        self.get_logger().info("left bin first msg: " + str(self._left_bin_first_msg))
        self.get_logger().info("right bin first msg: " + str(self._right_bin_first_msg))
        self.get_logger().info("table1 tray first msg: " + str(self._table1_tray_first_msg))
        self.get_logger().info("table2 tray first msg: " + str(self._table2_tray_first_msg))

        if self._left_bin_first_msg and self._right_bin_first_msg and self._table1_tray_first_msg and self._table2_tray_first_msg:
            self.get_logger().info('All trays received')
            self._order_logger()
            self._complete_kitting()
            self.get_logger().info('Kitting completed')
            self._kit_completed = True
        else:
            self.get_logger().info('Waiting for all trays to be received')
            return

'''
To test this script, run the following commands in separate terminals:

- ros2 launch robot_commander robot_commander.launch.py
- ros2 run robot_commander floor_robot_main.py
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=final
'''

import rclpy
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    '''
    Main function for the floor robot.
    '''
    rclpy.init(args=args)
    node = RobotCommanderInterface()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

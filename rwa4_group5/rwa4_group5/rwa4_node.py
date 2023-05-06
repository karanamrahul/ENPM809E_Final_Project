''' This module contains the Rwa4Node class for the RWA-4 assignment.'''


import rclpy
from rclpy.node import Node
from ariac_msgs.msg import *
import PyKDL
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy




class Rwa4Node(Node):
    ''' This class represents the RWA4 node for the RWA-4 assignment 

         Attributes:
            - subscription_orders : Subscription object for the "/ariac/orders" topic.
            - table1_camera_sub : Subscription object for the "/ariac/sensors/table1_camera/image" topic.
            - table2_camera_sub : Subscription object for the "/ariac/sensors/table2_camera/image" topic.
            - left_bin_camera_sub : Subscription object for the "/ariac/sensors/left_bins_camera/image" topic.
            - right_bin_camera_sub : Subscription object for the "/ariac/sensors/right_bins_camera/image" topic.
            - orders (List[Order]): A list to store the received orders.
            - table_dict (Dict[str, List[str]]): A dictionary to store the parts on each table.
            - part_list (List[Part]): A list to store the received parts.
            - table1_tray_first_msg (bool): A flag indicating whether the first message from table1 is received or not.
            - table2_tray_first_msg (bool): A flag indicating whether the first message from table2 is received or not.
            - left_bin_first_msg (bool): A flag indicating whether the first message from left bin camera is received or not.
            - right_bin_first_msg (bool): A flag indicating whether the first message from right bin camera is received or not.
            - order_print (bool): A flag indicating whether the order is printed or not.
            - colors (Dict[str, str]): A dictionary to store the colors of the parts.
            - part_types (Dict[str, str]): A dictionary to store the types of the parts.
            - everytgings_ready (bool): A flag indicating whether the order is ready or not.
            
            
            Methods:
            - order_callback(self, order): Callback function for the "/ariac/orders" topic.
            - table_camera_1_callback(self, tray_info): Callback function for the "/ariac/sensors/table1_camera/image" topic.
            - table_camera_2_callback(self, tray_info): Callback function for the "/ariac/sensors/table2_camera/image" topic.
            - left_bin_camera_callback(self, part_info): Callback function for the "/ariac/sensors/left_bins_camera/image" topic.
            - right_bin_camera_callback(self, part_info): Callback function for the "/ariac/sensors/right_bins_camera/image" topic.
            - logger(self): Function to print to the terminal.
            - _multiply_pose(self, pose1, pose2): Function to multiply two poses.
    '''

    def __init__(self):
        ''' Constructor for the Rwa4Node class'''
        # Initiate the Node class's constructor
        super().__init__('rwa4')
        
        # Subscriber for the "/ariac/orders" topic.
        self.subscription_orders = self.create_subscription(
            Order,
            '/ariac/orders',
            self._order_callback,
            10)
        
        # Subscriber for the "/ariac/sensors/table1_camera/image" topic.
        self.table1_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/table1_camera/image',
            self._table_camera_1_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT) # Need to set QOS profile due to conflict with other subscribers
        )

        # Subscriber for the "/ariac/sensors/table2_camera/image" topic.
        self.table2_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/table2_camera/image',
            self._table_camera_2_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscriber for the "/ariac/sensors/left_bins_camera/image" topic.
        self.left_bin_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/left_bins_camera/image',
            self._left_bin_camera_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscriber for the "/ariac/sensors/right_bins_camera/image" topic.
        self.right_bin_camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/right_bins_camera/image',
            self._right_bin_camera_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        
        self._orders = []  # List to store the received orders
        self._table_dict = {}  # Dictionary to store the parts on each table
        self._part_list = [] # List to store the parts
        
        # Flags for first messages 
        self._table1_tray_first_msg = False
        self._table2_tray_first_msg = False
        
        self._left_bin_first_msg = False
        self._right_bin_first_msg = False
        
        self._order_print = False     # Flag for printing the order
        
        self._everything_ready = False # Flag for checking if everything is ready  
        
        # Dictionaries for colors and part types
        self._colors = {"0": "Red", "1": "Green", "2": "Blue", "3": "Orange", "4": "Purple"}
        self._part_types = {"10": "Battery", "11":"Pump", "12": "Sensor", "13":"Regulator"}
        
        
        # self._orders_list = {}  # Dictionary to store the orders
        

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

    
    def _order_callback(self, order):
        ''' Callback function for the "/ariac/orders" topic.

            Args:
                - order (Order): The received order message.
        
        '''
        # Store the order in the list
        self._orders.append(self.OrderClass(order))
        self._orders.sort(key=lambda x: x.id)

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

        def __str__(self):
            ''' Overriding the __str__ method to print the tray information'''
            return f"Tray ID: {self.id}, Pose: {self.pose}, Sensor Pose: {self.sensor_pose}"
        
    def _table_camera_1_callback(self, tray_info):
        
        ''' Callback function for the "/ariac/sensors/table1_camera/image" topic.
        
            Args:
                - tray_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/table1_camera/image" topic.
                
        '''
         
        if( (not self._table1_tray_first_msg) and (len(tray_info.tray_poses) > 0)): # Check if the tray is empty
            self.table1_tray = self.Tray(tray_info)
            
            converted_pose = self._multiply_pose(self.table1_tray.sensor_pose, self.table1_tray.pose) # Convert the pose of the tray to the world frame
            
            self.table1_tray.pose = converted_pose
            
            # Add to dictionary
            self._table_dict[str(self.table1_tray.id)] = self.table1_tray 
            self._table1_tray_first_msg = True
        
        elif not self._order_print:
            self._order_logger() 
        else:
            pass
        
    def _table_camera_2_callback(self, tray_info):
    
        
        ''' Callback function for the "/ariac/sensors/table2_camera/image" topic.

            Args:
                - tray_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/table2_camera/image" topic.
        
        '''
        
        
        if ((not self._table2_tray_first_msg) and (len(tray_info.tray_poses) > 0)):
            self.table2_tray = self.Tray(tray_info)
            
            converted_pose = self._multiply_pose(self.table2_tray.sensor_pose, self.table2_tray.pose) # Convert the pose of the tray to the world frame
            
            self.table2_tray.pose = converted_pose
            
            # Add to dictionary
            self._table_dict[str(self.table2_tray.id)] = self.table2_tray
            self._table2_tray_first_msg = True
            
        elif not self._order_print: # Check if the order has been printed
            self._order_logger() 
            
        else:
            pass
        
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
            

        # Overload the __str__ method to check the attributes of the class
        def __str__(self):
            ''' Overriding the __str__ method to print the part information'''
            return f"Sensor Pose: {self.sensor_pose}, Parts: {self.parts}"
        
    def _left_bin_camera_callback(self, part_info):
        ''' Callback function for the "/ariac/sensors/left_bins_camera/image" topic.
        
            Args:
                - part_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/left_bins_camera/image" topic.
            
            '''
            
        if not self._left_bin_first_msg:
            self.left_bin = self.Part(part_info)
            
            for i in range(len(self.left_bin.parts)):
                part_info = {
                    'type': self.left_bin.parts[i].part.type,
                    'color': self.left_bin.parts[i].part.color,
                    'pose': self.left_bin.parts[i].pose
                }
                part_info["pose"] = self._multiply_pose(self.left_bin.sensor_pose, part_info['pose'])

                self._part_list.append(part_info)
                # print(self.part_list)
            
            self._left_bin_first_msg = True
            
            
        elif not self._order_print:
            self._order_logger() 
        else:
            pass
        
    def _right_bin_camera_callback(self, part_info):
        ''' Callback function for the "/ariac/sensors/right_bins_camera/image" topic. 
        
        
            Args:
                - part_info (AdvancedLogicalCameraImage): The message received from the "/ariac/sensors/right_bins_camera/image" topic.
        '''
        
        
        if not self._right_bin_first_msg:
            self.right_bin = self.Part(part_info)
            
            for i in range(len(self.right_bin.parts)):
                part_info = {
                    'type': self.right_bin.parts[i].part.type,
                    'color': self.right_bin.parts[i].part.color,
                    'pose': self.right_bin.parts[i].pose
                }
                part_info["pose"] = self._multiply_pose(self.right_bin.sensor_pose, part_info['pose'])

                self._part_list.append(part_info)
                # print(self.part_list)
               
            self._right_bin_first_msg = True
        
        elif not self._order_print:
            self._order_logger() 
        else:
            pass
        
        
    def _order_logger(self):
        
        
        ''' This function prints the order information,tray pose and the part information.'''
        
        print("Issue with the order")
        print(self._orders)
        
        tray_id = str(self._orders[0].tray_id)
        part_ids = self._orders[0].parts
        
                
        # print(self.part_list)
        # print(tray_id)
        
        parts_needed = [False]*len(self._part_list)
        
        for i in range(len(part_ids)):
            part_color = part_ids[i]["color"]
            part_type = part_ids[i]["type"]
            
            for j in range(len(self._part_list)):
                if self._part_list[j]["color"] == part_color and self._part_list[j]["type"] == part_type:
                    parts_needed[j] = True 
        
        if (tray_id in self._table_dict.keys()) and all(parts_needed):
            self._everything_ready = True
        
        if self._everything_ready:
            
            self._order_print = True
            
            print(" ----------------------")
            print(" --- Order " + str(self._orders[0].id) + " ---")
            print(" ----------------------")
            
            
            print(" Tray: \n")
            
            
            # Get the tray pose from the dictionary
            table_position = self._table_dict[tray_id].pose.position
            table_orientation = self._table_dict[tray_id].pose.orientation
            table_position_list = [table_position.x, table_position.y, table_position.z]
            table_orientation_list = [table_orientation.x, table_orientation.y, table_orientation.z, table_orientation.w]
            

            print("\t- id: " + str(self._table_dict[tray_id].id))
            print("\t- pose: ")
            print("\t\t- position: " + str(table_position_list))
            print("\t\t- orientation: " + str(table_orientation_list))

            
            print(" Part:")
            for i in range(len(part_ids)):
                part_color = part_ids[i]["color"]
                part_type = part_ids[i]["type"]
                
                for j in range(len(self._part_list)):
                    if self._part_list[j]["color"] == part_color and self._part_list[j]["type"] == part_type:
                        part_pose = self._part_list[j]["pose"]
                        part_position = part_pose.position
                        part_orientation = part_pose.orientation
                        part_position_list = [part_position.x, part_position.y, part_position.z]
                        part_orientation_list = [part_orientation.x, part_orientation.y, part_orientation.z, part_orientation.w]
                        
                        print("\t- " + self._colors[str(part_color)] + " " + self._part_types[str(part_type)])
                        print("\t\t- pose: ")
                        print("\t\t\t- position: " + str(part_position_list))
                        print("\t\t\t- orientation: " + str(part_orientation_list))
                        print("\n")
                        break
            

        
            
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

def main(args=None):
    ''' Main function for the node in ROS2.'''
    
    rclpy.init(args=args) # initialize the node
    node = Rwa4Node() # create the node 
    rclpy.spin(node) # spin the node 
    rclpy.shutdown() # shutdown the node

if __name__ == '__main__':
    main()



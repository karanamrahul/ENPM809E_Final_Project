import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from competitor_interfaces.msg import Robots as RobotsMsg
from competitor_interfaces.srv import (
    EnterToolChanger , ExitToolChanger,PlacePartInTray,PickupTray,MoveTrayToAGV,PlaceTrayOnAGV,RetractFromAGV,PickupPart,MovePartToAGV,MoveAGV)
from ariac_msgs.msg import Part
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import quaternion_from_euler




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

        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        # Flag to indicate if the kit has been completed
        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None

        # subscriber
        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)

        # timer
        self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)

        # Service client for starting the competition
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',
            callback_group=service_group)
        
        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=service_group)
        
        # Service client for entering the gripper slot
        self.retract_from_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',
            callback_group=service_group)
        
        # Service Client for placing the part in the tray
        self.place_part_in_tray_client = self.create_client(PlacePartInTray,'/competitor/floor_robot/place_part_in_tray',
                                                  callback_group=service_group   )

        # Service Client for picking up the tray
        self.pickup_tray_client = self.create_client(PickupTray,'/competitor/floor_robot/pickup_tray',callback_group=service_group )




        # Service Client for moving the tray to agv
        self.move_tray_to_agv_client=self.create_client(MoveTrayToAGV,'/competitor/floor_robot/move_tray_to_agv',callback_group=service_group)


        # Service Client for placing the tray on agv
        self.place_tray_on_agv_client = self.create_client(PlaceTrayOnAGV,'/competitor/floor_robot/place_tray_on_agv',callback_group=service_group)

        # Service Client for retracting the tray from agv
        self.retract_from_agv_client = self.create_client(RetractFromAGV,'/competitor/floor_robot/retract_from_agv',callback_group=service_group)


        # Service Client for picking up the part
        self.pickup_part_client = self.create_client(PickupPart,'/competitor/floor_robot/pickup_part',callback_group=service_group)
   
   
        # Service Client for moving the part to agv
        self.move_part_to_agv_client = self.create_client(MovePartToAGV,'/competitor/floor_robot/move_part_to_agv',callback_group=service_group)
   
        # Service Client for locking the tray and parts on the agv (TODO: Need to update X with the agv number)
        self.lock_agv_client= self.create_client(Trigger,'/ariac/agv'+'X' +'_lock_tray',callback_group=service_group)
        
        # Service Client for moving the agv to warehosue  (TODO: Need to update X with the agv number)
        self.move_agv_client.call_async = self.create_client(MoveAGV,' /ariac/move_agv'+'X',callback_group=service_group)
    def _competition_state_cb(self, msg: CompetitionState):
        '''
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        '''
        self._competition_state = msg.competition_state

    # def _robot_action_timer_callback(self):
    #     '''
    #     Callback for the timer that triggers the robot actions
    #     '''

    #     if self._competition_state == CompetitionState.READY and not self._competition_started:
    #         self.start_competition()

    #     # exit the callback if the kit is completed
    #     if self._kit_completed:
    #         return

    #     # move robot home
    #     self.move_robot_home("floor_robot")

    #     # change gripper type
    #     self.goto_tool_changer("floor_robot", "kts1", "trays")

    #     # to ignore function calls in this callback
    #     self._kit_completed = True

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

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')
    
    
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
        request.tray_table = tray_table

        future = self.pickup_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
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
    def place_tray(self, robot, agv):
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

        future = self.place_tray_on_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Tray placed on AGV')
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
            part_color (str): The color of the part.
            bin_id (str): The id of the bin.

        Raises:
            ValueError: If the robot name is invalid.
        """
        request = PickupPart.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.part_type = part_type
        request.part_color = part_color
        request.bin_id = bin_id

        future = self.pickup_part_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
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
        request.agv_id = agv
        request.destination_id = 'ASRS'  # Set the AGV's destination to the warehouse
        request.location = MoveAGV.WAREHOUSE  # Set the location to the warehouse

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

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''
        
        if self._kit_completed:
            return

        # Pose of tray in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        tray_pose = Pose()
        tray_pose.position.x = -0.87
        tray_pose.position.y = -5.84
        tray_pose.position.z = 0.73499
        tray_pose.orientation.x = quaternion[0]
        tray_pose.orientation.y = quaternion[1]
        tray_pose.orientation.z = quaternion[2]
        tray_pose.orientation.w = quaternion[3]

        # Pose of a purple pump in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        purple_pump_pose = Pose()
        purple_pump_pose.position.x = -2.08
        purple_pump_pose.position.y = 2.805001
        purple_pump_pose.position.z = 0.723487
        purple_pump_pose.orientation.x = quaternion[0]
        purple_pump_pose.orientation.y = quaternion[1]
        purple_pump_pose.orientation.z = quaternion[2]
        purple_pump_pose.orientation.w = quaternion[3]

        pump_type = Part.PUMP
        pump_color = Part.PURPLE
        pump_quadrant = 2
        pump_bin = "right_bins"

        # Pose of a blue battery in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        blue_battery_pose = Pose()
        blue_battery_pose.position.x = -2.080001
        blue_battery_pose.position.y = -2.445
        blue_battery_pose.position.z = 0.723434
        blue_battery_pose.orientation.x = quaternion[0]
        blue_battery_pose.orientation.y = quaternion[1]
        blue_battery_pose.orientation.z = quaternion[2]
        blue_battery_pose.orientation.w = quaternion[3]

        battery_type = Part.BATTERY
        battery_color = Part.BLUE
        battery_quadrant = 4
        battery_bin = "left_bins"

        # ID of the tray
        tray_id = 3

        # table where the tray is located
        tray_table = "kts1"

        # AGV
        agv = "agv1"
         # move robot home
        self.move_robot_home("floor_robot")

        # change gripper type
        self.goto_tool_changer("floor_robot", tray_table, "trays")
        self.retract_from_tool_changer("floor_robot", tray_table, "trays")

        # pick and place tray
        self.pickup_tray("floor_robot", tray_id, tray_pose, tray_table)
        self.move_tray_to_agv("floor_robot", tray_pose, agv)
        self.place_tray("floor_robot", tray_id, agv)
        self.retract_from_agv("floor_robot", agv)

        # change gripper to pick up parts
        self.goto_tool_changer("floor_robot", "kts2", "parts")
        self.retract_from_tool_changer("floor_robot", "kts2", "parts")

        # pick and place purple pump
        self.pickup_part("floor_robot", purple_pump_pose, pump_type, pump_color, "right_bins")
        self.move_part_to_agv("floor_robot", purple_pump_pose, agv, pump_quadrant)
        self.place_part_in_tray("floor_robot", agv, pump_quadrant)
        self.retract_from_agv("floor_robot", agv)

       # move robot home
        self.move_robot_home("floor_robot")

        # change gripper type
        self.goto_tool_changer("floor_robot", tray_table, "trays")
        self.retract_from_tool_changer("floor_robot", tray_table, "trays")

        # pick and place tray
        self.pickup_tray("floor_robot", tray_id, tray_pose, tray_table)
        self.move_tray_to_agv("floor_robot", tray_pose, agv)
        self.place_tray("floor_robot", tray_id, agv)
        self.retract_from_agv("floor_robot", agv)

        # change gripper to pick up parts
        self.goto_tool_changer("floor_robot", "kts2", "parts")
        self.retract_from_tool_changer("floor_robot", "kts2", "parts")

        # pick and place purple pump
        self.pickup_part("floor_robot", purple_pump_pose, pump_type, pump_color, "right_bins")
        self.move_part_to_agv("floor_robot", purple_pump_pose, agv, pump_quadrant)
        self.place_part_in_tray("floor_robot", agv, pump_quadrant)
        self.retract_from_agv("floor_robot", agv)

        # move robot home
        self.move_robot_home("floor_robot")

        # pick and place blue battery
        self.pickup_part("floor_robot", blue_battery_pose, battery_type, battery_color, battery_bin)
        self.move_part_to_agv("floor_robot", blue_battery_pose, agv, battery_quadrant)
        self.place_part_in_tray("floor_robot", agv, battery_quadrant)
        self.retract_from_agv("floor_robot", agv)

        # move robot home
        self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.lock_agv(agv)
        self.move_agv_to_warehouse(agv)

        self._kit_completed = True


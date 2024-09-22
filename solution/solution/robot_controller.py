import sys
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, GetCostmap
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from auro_interfaces.msg import StringWithPose
from assessment_interfaces.msg import HomeZone, ItemList, ItemHolders, ItemLog, ItemHolder, RobotList, Item
from solution_interfaces.msg import ItemMark, ItemMarks, IsNavigating
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import angles
import time

from enum import Enum

#Global variables:

LINEAR_VELOCITY  = 0.3 # Metres per second , caps maximum movement speed
ANGULAR_VELOCITY = 0.55 # Radians per second , caps maximum turn speen
COLLECTION_DISTANCE_THRESHOLD = 0.5 # Meters, adjust as needed
IMU_THRESHOLD = 0.1 #Angular velocity of IMU data across x,y
ITEM_DIAMETER_THRESHOLD = 20 #Diameter of items to filter in from image data
SCAN_COMPLETION_THRESHOLD = 0.0872 #Radians, stop scanning if threshold reached

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.65 # Metres per second 0.55
SIDE_SCAN_THRESHOLD = 0.6 # Metres per second 0.3
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3


# Finite state machine (FSM) states
class State(Enum):
    FORWARD = 0
    TURNING = 1
    SET_GOAL = 2
    COLLECTING = 3
    RETURNING = 4
    NAVIGATING = 5
    RECOVERY = 6
    ONWARD = 7
    WAITFORSWAP = 8
    GO_TO_WAYPOINT = 9
    SCAN = 10
    TURN = 11
    DECIDE_ACTION = 12

def diameter_to_scan(diameter):
        scan_length = 2.40649708 - 0.0122437844 * diameter + 0.0000190369561 * diameter**2
        return scan_length

def inf_to_num(x):
    if math.isinf(x):
        return 
    else:
        return x

class RobotController(Node):


    

    def __init__(self):
        super().__init__('robot_controller')

        #----------------------------------------- INITIALIZATION-----------------------------------------------------------------------------------------------

        # Class variables used to store persistent values between executions of callbacks and control loop

        self.state = State.SET_GOAL                      # Current FSM state
        self.previous_state = State.FORWARD            # Previous FSM state to keep track of state: NEW WIP - GOING BACK TO PREVIOUS STATES AFTER COLLISION / RECOVERY
        self.id = self.get_namespace()                  # Get id of each individual robot
        self.pose = Pose()                              # Current pose (position and orientation), relative to the odom reference frame
        self.previous_pose = Pose()                     # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0                                  # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0                         # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0                           # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_RIGHT                 # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(2.0, 3.0) 
        #self.goal_distance = random.uniform(1.0, 2.0)   # Goal distance to travel in FORWARD state
        self.scan_triggered = [False] * 4               # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD
        




        #SOLUTION TOPICS:
        self.items = ItemList()
        self.homezone = HomeZone()
        self.itemholder = ItemHolder()
        self.itemholders = ItemHolders()
        self.itemlog = ItemLog()
        self.estimated_distance = 0.0


        self.item = Item()
        self.counter = 0
        self.previous_item = Item()
        self.robot = RobotList()
        self.previous_turn_direction = TURN_LEFT
        

        self.mark_counter = 0

        self.item_yaw_pairs = []
        self.yawmark = 0
        self.target_yaw = 0.0
        self.turn_count = 0
        self.crosspos = False
        self.crossneg = False
        self.target = 0


        #LOCALIZATION: ODOM
        self.x_to_odom = 0.0
        self.y_to_odom = 0.0            
        self.distance_to_odom = 0.0
        self.angle_to_odom = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.returny = self.initial_y

        #LOCALIZATION: MAP
        self.x = 0.0
        self.y = 0.0
        self.distance = 0.0 #distance from goal
        self.angle = 0.0 #angle from goal


        self.mapx = 0.0
        self.mapy = 0.0
        self.mapyaw = 0.0
        self.mapdistance = 0.0 #distance from goal
        self.mapangle = 0.0 #angle from goal
        

        #IMU TOPICS
        self.imu = Imu()

        #-------------------------------------------------INITIALIZATION: NAV2 --------------------------------------------------------------------------------------------

        #NAV2 SETUP
        self.navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y

        (initial_pose.pose.orientation.x,
         initial_pose.pose.orientation.y,
         initial_pose.pose.orientation.z,
         initial_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        

        #ITEMMARKS
        self.blue_marks = ItemMarks()
        self.blue_mark_goal = PoseStamped() #ball value, ball mapx, mapy, ball distance from home, robot who collected from
        self.blue_marks_count = 0

        self.marks = ItemMarks()
        self.goalmark = ItemMark() #FINAL
        self.goalmark.value = 0

        self.red_marks = ItemMarks()
        self.red_mark_goal = PoseStamped() #ball value, ball mapx, mapy, ball distance from home, robot who collected from
        self.red_marks_count = 0

        self.previous_mark = ItemMark()


        #-------------------------------------------------------------------SUBSCRIBERS----------------------------------------------------------------------------------------------------------
        
        #MARKS SUBSCRIBER
        self.itemmarks_subscriber = self.create_subscription(
            ItemMarks,
            '/item_marks',
            self.itemmarks_callback,
            10
        )
        
        
        #ITEM SUBSCRIBER
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )

        #HOMEZONE SUBSCRIBER
        self.homezone_subscriber = self.create_subscription(
            HomeZone,
            'home_zone',
            self.homezone_callback,
            10
        )

        #ITEMHOLDER SUBSCRIBER
        self.itemholder_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.itemholder_callback,
            10
        )

        #ITEMLOG SUBSCRIBER
        self.itemlog_subscriber = self.create_subscription(
            ItemLog,
            '/item_log',
            self.itemlog_callback,
            10
        )

        # Subscribes to Odometry messages published on /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Subscribes to LaserScan messages on the /scan topic #revised scan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan', 
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        
        # Subscribes to LaserScan messages on the /scan topic
        self.robot_subscriber = self.create_subscription(
            RobotList,
            'robots',
            self.robot_callback,
            10)
        
        # Subscribes to LaserScan messages on the /scan topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        
        
        
        
        #-------------------------------------------------------------------------PUBLISHERS-----------------------------------------------------------------------------------------------
        
        # Publishes Twist messages (linear and angular velocities) on the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publishes custom StringWithPose (see auro_interfaces/msg/StringWithPose.msg) messages on the /marker_input topic
        self.marker_publisher = self.create_publisher(StringWithPose, 'marker_input', 10)

        #MARK publisher
        self.itemmarks_publisher = self.create_publisher(ItemMarks, '/item_marks', 10)

        #NAV STATE publisher
        self.isnavigating_publisher = self.create_publisher(IsNavigating, 'is_navigating', 10)

        # Creates a timer that calls the control_loop method repeatedly - each loop represents single iteration of the FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.previous_timer = self.get_clock().now() #

    #-------------------------------------------------------------------------------CALLBACK FUNCTIONS------------------------------------------------------------------------------------
    
    def itemmarks_callback(self, msg): #new swap
        self.marks.data = sorted(msg.data, key=lambda mark: mark.dist_from_home, reverse=True)

        #self.red_marks.data = [m for m in msg.data if (m.value == 5 and m.x < 0.0)]
        self.blue_marks.data = [m for m in msg.data if (m.value == 15)] # and m.x > 0.0
    

    def item_callback(self, msg):
        self.items = msg

    def robot_callback(self, msg):
        self.robot = msg
    
    def homezone_callback(self, msg):
        self.homezone = msg

    def imu_callback(self, msg):
        self.imu = msg

    def itemholder_callback(self, msg): #have to loop and find / ordering different everytime
        sorted_itemholders = ItemHolders()
        sorted_itemholderlist = sorted(msg.data, key=lambda x: int(x.robot_id.replace("robot", "")))
        sorted_itemholders.data = sorted_itemholderlist


        if self.id == '/robot1': #the values are flipped: robot 2 is first on the list
            self.itemholder = sorted_itemholders.data[0] #need some way to discern item holder
        elif self.id == '/robot2':
            self.itemholder = sorted_itemholders.data[1] 
        else:
            self.itemholder = sorted_itemholders.data[2] 


    def itemlog_callback(self, msg):
        self.itemlog = msg



    def robot_callback(self, msg):
        self.robot.data =  sorted(msg.data, key=lambda robot: robot.size, reverse=True)
    
    #---------------------------------------------------------------------BASE CALLBACKS----------------------------------------------------------------------------------------------------

    # Called every time odom_subscriber receives an Odometry message from the /odom topic
    # The pose estimates are expressed in a coordinate system relative to the starting pose of the robot
    def odom_callback(self, msg):
        self.pose = msg.pose.pose # Store the pose in a class variable
        # Roll (rotation around X axis) and pitch (rotation around Y axis) are discarded
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw # Store the yaw in a class variable
        

    # Called every time scan_subscriber recieves a LaserScan message from the /scan topic
    def scan_callback(self, msg):
            
            # Group scan ranges into 4 segments
            # Front, left, and right segments are each 60 degrees
            # Back segment is 180 degrees


            front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
            left_ranges  = msg.ranges[31:60] # 31 to 90 degrees (31 to 90 degrees)
            right_ranges = msg.ranges[300:330] # 271 to 330 degrees (-30 to -91 degrees)
            back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)


            # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
            self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
            self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
            self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SIDE_SCAN_THRESHOLD 
            self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SIDE_SCAN_THRESHOLD


        

    def control_loop(self):

        # Send message to rviz_text_marker node
        marker_input = StringWithPose()
        marker_input.text = str(self.state) + " +" + str(self.id) # Visualise robot state as an RViz marker
        marker_input.pose = self.pose # Set the pose of the RViz marker to track the robot's pose
        self.marker_publisher.publish(marker_input)

        self.is_navigating = IsNavigating()
        self.is_navigating.isnavigating = (True if self.state == State.NAVIGATING else False)
        self.isnavigating_publisher.publish(self.is_navigating)
        self.get_logger().info(f"STATE: {self.state}")
        self.get_logger().info(f"STATE: {self.is_navigating.isnavigating}")

        def detect_robots():
            if len(self.robot.data) > 0:
                closest_robot = self.robot.data[0]
                if closest_robot.y <= 25 or closest_robot.size >= 0.30:
                    return True
                else:
                    return False
            else:
                return False
        
        def decide_robot_turn():
            if len(self.robot.data) > 0:
                closest_robot = self.robot.data[0]    
                robot_turn_direction = TURN_LEFT if closest_robot.x <= 0 else TURN_RIGHT
                robot_turn_angle = abs(closest_robot.x)/ 320
                return robot_turn_direction, robot_turn_angle
            
                 

        #TRANSFORMATIONS:
        target_frame = 'base_link'
        source_frame = 'odom'

        #ODOM TRANSFORMATION
        try:
            t = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time())
            
            self.x_to_odom = t.transform.translation.x
            self.y_to_odom = t.transform.translation.y

            self.distance_to_odom = math.sqrt(self.x_to_odom ** 2 + self.y_to_odom ** 2)
            self.angle_to_odom = math.atan2(self.y_to_odom, self.x_to_odom)


        except TransformException as e:
            self.get_logger().info(f"{e}")

        #MAP TRANSFORMATION
        try:
            r = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            
            self.mapx = r.transform.translation.x
            self.mapy = r.transform.translation.y

            (roll, pitch, self.mapyaw) = euler_from_quaternion([r.transform.rotation.x,
                                                        r.transform.rotation.y,
                                                        r.transform.rotation.z,
                                                        r.transform.rotation.w])

            self.mapdistance = math.sqrt(self.mapx ** 2 + self.mapy ** 2)
            self.mapangle = math.atan2(self.mapy, self.mapx)

        except TransformException as e:
            self.get_logger().info(f"{e}")



            
        
            
        #-------------------------------------------------------MAIN CODE: FSM -----------------------------------------------------------------------------------------------------------------------
        match self.state:

            case State.GO_TO_WAYPOINT:

                    self.goalmark = self.blue_marks.data[0]
                    self.get_logger().info(f"BLUE MARKER UPDATED - X: {self.goalmark.x:.2f}")
                    self.get_logger().info(f"BLUE MARKER UPDATED - Y: {self.goalmark.y:.2f}")


                    self.get_logger().info("navigating to new BLUE marked FOR SWAP")

                    self.blue_mark_goal.header.frame_id = 'map'
                    self.blue_mark_goal.header.stamp = self.get_clock().now().to_msg()
                    self.blue_mark_goal.pose.position.x = self.goalmark.x 
                    self.blue_mark_goal.pose.position.y = self.goalmark.y 

                    (self.blue_mark_goal.pose.orientation.x,
                    self.blue_mark_goal.pose.orientation.y,
                    self.blue_mark_goal.pose.orientation.z,
                    self.blue_mark_goal.pose.orientation.w) = quaternion_from_euler(0, 0, self.goalmark.yaw , axes='sxyz')
                    

                    self.navigator.goToPose(self.blue_mark_goal)
                    self.previous_state = State.GO_TO_WAYPOINT
                    self.state = State.NAVIGATING 
                    return 

                    

            case State.FORWARD:

                if (self.imu.angular_velocity.x > IMU_THRESHOLD) or (self.imu.angular_velocity.y > IMU_THRESHOLD):
                    self.state = State.RECOVERY
    
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 30 

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT] and self.scan_triggered[SCAN_FRONT]:

                        self.turn_direction = self.previous_turn_direction
                        self.turn_angle = 45 
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.previous_turn_direction = self.turn_direction
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: 
                        self.turn_direction = TURN_LEFT
                        self.previous_turn_direction = self.turn_direction
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                
                if self.scan_triggered[SCAN_FRONT]:
                    self.turn_direction = self.previous_turn_direction
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45 
                    
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                
                if detect_robots():
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_direction, self.turn_angle = decide_robot_turn()
                    self.previous_turn_direction = self.turn_direction
                    self.get_logger().info(f"Detected ROBOT TO " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" TURNING by {self.turn_angle:.2f} degrees")
                    return
                
                if  any([(i.value == 10 and i.diameter >= 60) for i in self.items.data]):
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 30 #45 NEW 29TH 90
                    self.turn_direction = self.previous_turn_direction
                    return
 
            
                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)


                if self.itemholder.holding_item == True and self.itemholder.item_value == 15: 
                    self.get_logger().info(f"FORWARD TO RETURN STATE")
                    self.state = State.RETURNING
                    return
                
                if self.itemholder.item_value == 5 and len(self.blue_marks.data) > 0:
                    self.get_logger().info(f"FORWARD TO GO_TO_WAYPOINT STATE")
                    self.state = State.GO_TO_WAYPOINT

                if len(self.items.data) > 0 and any([i.value == self.target for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]):   #DIAMETER THRESHOLD                   
                    self.state = State.COLLECTING
                    return  

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                if distance_travelled >= self.goal_distance: 
                    self.previous_yaw = self.yaw
                    self.state = State.SCAN
                    return 
                    

            case State.TURNING: #turn other way if starting to look backwards
                # new recovery
                if (self.imu.angular_velocity.x > IMU_THRESHOLD) or (self.imu.angular_velocity.y > IMU_THRESHOLD):
                    #self.initial_state = self.state new imu
                    self.state = State.RECOVERY

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)
                self.get_logger().info(f"TURNING: {self.turn_direction}")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)    
                self.get_logger().info(f"Turned {math.degrees(math.fabs(yaw_difference)):.2f} out of {self.turn_angle:.2f} degrees")  

            
                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)

                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")
                    return 
                
                 
                if self.itemholder.holding_item == True and self.itemholder.item_value == 15:
                    self.get_logger().info(f"TURN TO RETURN STATE")
                    self.state = State.RETURNING
                    return
                
                if self.itemholder.item_value == 5 and len(self.blue_marks.data) > 0:
                    self.get_logger().info(f"FORWARD TO GO_TO_WAYPOINT STATE")
                    self.state = State.GO_TO_WAYPOINT


                if len(self.items.data) > 0 and any([i.value == self.target for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]):                   
                    self.state = State.COLLECTING
                    return          

            
            case State.RECOVERY:
                

                #if robot collides
                if (self.imu.angular_velocity.x > IMU_THRESHOLD) or (self.imu.angular_velocity.y > IMU_THRESHOLD):
                    stop_msg = Twist()
                    stop_msg.linear.x = 0.0
                    stop_msg.angular.z = 0.0
                    self.cmd_vel_publisher.publish(stop_msg)

                    #IMPLEMENT Sleep
                    time.sleep(10)

                    #reinitialize pose
                    #clear local costmap
                    self.navigator.clearLocalCostmap()
    
                    recovery_pose = PoseStamped()
                    recovery_pose.header.frame_id = 'map'
                    recovery_pose.header.stamp = self.get_clock().now().to_msg()
                    recovery_pose.pose.position.x = self.mapx
                    recovery_pose.pose.position.y = self.mapy

                    (recovery_pose.pose.orientation.x,
                    recovery_pose.pose.orientation.y,
                    recovery_pose.pose.orientation.z,
                    recovery_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(self.mapyaw), axes='sxyz')


                    self.navigator.setInitialPose(recovery_pose)
                    self.navigator.waitUntilNav2Active()
                    
                    
                    return
                else:
                    msg = Twist()
                    msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                    self.cmd_vel_publisher.publish(msg)
                    self.state = State.FORWARD 
                    return
                
            
                
                              
            case State.SET_GOAL: #CHECK IF AVAILABLE

                if not self.itemholder.holding_item:
                    if self.itemlog.blue_count >= 1: #MINIMUM BLUE SWAPS
                        if len(self.items.data) > 0 and any([i.value == 15 for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]): 
                            self.target = 15
                        else:
                            self.target = 5
                    else:
                        self.target = 5
                else:
                    if self.itemholder.item_value == 15:
                            self.state = State.RETURNING
                            return
                    elif self.itemholder.item_value == 5:
                            self.target = 15
                            self.get_logger().info(f"TARGET updatred: {self.target}")
                    else:
                            self.target = 5
                
                self.state = State.DECIDE_ACTION
                return


            case State.DECIDE_ACTION: #DECIDES ACTION - GOTO WAYPOINT / COLLECT / SEARCH

                if self.itemholder.item_value == 5 and len(self.blue_marks.data) > 0:
                    self.state = State.GO_TO_WAYPOINT
                    return

                if len(self.items.data) > 0 and any([i.value == self.target for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]):               
                        self.state = State.COLLECTING
                        return
                else:
                    if self.previous_state == State.NAVIGATING or self.target == 5:
                        self.previous_yaw = self.yaw
                        self.state = State.SCAN
                        return
                    
                    if self.previous_state == State.ONWARD:
                        self.state = State.FORWARD
                        return
                
            case State.SCAN:

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)
                self.get_logger().info(f"SCANNING")

                if len(self.items.data) > 0 and any([i.value == self.target for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]): 
                        self.state = State.DECIDE_ACTION
                        self.crosspos = False
                        self.crossneg = False
                        return

                if self.yaw > 0:
                    self.crosspos = True
                if self.yaw < 0:
                    self.crossneg = True

                if self.crossneg and self.crosspos and abs(self.yaw - self.previous_yaw) < SCAN_COMPLETION_THRESHOLD:
                    self.get_logger().info(f"SCANNING COMPLETE")
                    self.crosspos = False
                    self.crossneg = False
                    self.get_logger().info(f"TARGET YAW REACHED - COLLECTING")
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    return

            case State.COLLECTING:                    

                #IMU COLLISION RECOVERY:
                if (self.imu.angular_velocity.x > IMU_THRESHOLD) or (self.imu.angular_velocity.y > IMU_THRESHOLD):
                    self.state = State.RECOVERY
                    return


                #COLLISION AVOIDANCE:
                if (any([(i.value == 10 and i.diameter >= 60) for i in self.items.data])) or detect_robots() or self.scan_triggered[SCAN_FRONT] or self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.get_logger().info(f"OBSTACLE DETECTED WHILE COLELCTING")
                    self.state = State.FORWARD
                    return

                #PUBLISH WAYPOINT
                if self.mark_counter == 0:
                    if all([i.value == 15 for i in self.items.data]) and any([i.diameter >= 50 for i in self.items.data]) and len(self.items.data) >= 2 and not(self.scan_triggered[SCAN_FRONT] or self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]): #4 ITEMS >=50 og DIAMETER - 40
                        mark = ItemMark()                        
                        mark.x = self.mapx
                        mark.y = self.mapy
                        mark.robot_id = self.id
                        mark.colour = self.itemholder.item_colour 
                        mark.value = 15
                        mark.dist_from_home = self.distance_to_odom
                        mark.yaw = self.yaw

                        if abs(self.previous_mark.x - mark.x) >= 0.2: # filter duplicates
                            self.previous_mark = mark
                            self.marks.data.append(mark)
                            self.itemmarks_publisher.publish(self.marks)  
                            self.get_logger().info(f"COLLECTED ITEM MARKED") 

                            self.mark_counter += 1
                


                if len(self.items.data) == 0 and self.estimated_distance < COLLECTION_DISTANCE_THRESHOLD:
                    self.get_logger().info(f"APPROACHED ITEM")  
                    self.previous_pose.position.x = self.pose.position.x
                    self.previous_item.value = self.itemholder.item_value

                    self.state = State.ONWARD
                    return
                
                
                
                
                if (len(self.items.data) > 0 and self.itemholder.holding_item == False) or (len(self.items.data) > 0 and self.itemholder.holding_item == True and self.itemholder.item_value < 15):
                    
                    #UPDATE GOAL:
                    if not self.itemholder.holding_item:
                        if self.itemlog.blue_count >= 1: 
                            if len(self.items.data) > 0 and any([i.value == 15 for i in self.items.data if i.y <= 10 and i.diameter > ITEM_DIAMETER_THRESHOLD]): 
                                self.target = 15
                            else:
                                self.target = 5
                        else:
                            self.target = 5
                    else:
                        if self.itemholder.item_value == 15:
                                self.state = State.RETURNING
                                return
                        elif self.itemholder.item_value == 5:
                                self.target = 15
                                self.get_logger().info(f"TARGET updatred: {self.target}")
                        else:
                                self.target = 5

                    if not self.itemholder.holding_item:

                            sorted_data = sorted(self.items.data, key=lambda item: item.value * item.diameter, reverse=True) 
                            sorted_items = ItemList()
                            sorted_items.data = sorted_data 

                            filtered_robot_items = [i for i in sorted_items.data if i.y <= 10 and (i.value == self.target) and i.diameter > ITEM_DIAMETER_THRESHOLD]

                    else:
                        #SORTS TO PRIORITIZE CLOSEST BLUE
                        sorted_data = sorted(self.items.data, key=lambda item: item.diameter, reverse=True)
                        sorted_items = ItemList()
                        sorted_items.data = sorted_data 
                        filtered_robot_items = [i for i in sorted_items.data if i.y <= 10 and (i.value == self.target) and i.diameter > ITEM_DIAMETER_THRESHOLD]
                    
                
                    if len(filtered_robot_items) > 0:                        
                        self.item = filtered_robot_items[0] 
                    else:
                        self.get_logger().info(f"NO BETTER ITEMS SEEN - FORWARD")
                        self.state = State.FORWARD 
                        return
                    

                    self.estimated_distance = (69.0 * float(self.item.diameter) ** -0.89) 

                    estimated_angle = self.item.x/320.0  # Assuming 320 is the half-width of the image
                    

                    self.get_logger().info(f"Estimated DISTANCE to item {self.estimated_distance:.2f} metres from 0.1")

                    msg = Twist()
                    msg.linear.x = min(self.estimated_distance, LINEAR_VELOCITY) # Cap the velocity
                    msg.angular.z = estimated_angle * ANGULAR_VELOCITY
                    self.cmd_vel_publisher.publish(msg)
                    return
                             

            case State.ONWARD:
                if len(self.items.data) == 0:
                    
                    if self.item.value == 5:
                        self.returny = self.mapy
                        self.get_logger().info(f"RECALIBRATE Y RETURN = {self.returny}")
                    
                    if self.itemholder.item_value == self.item.value: 
                        self.get_logger().info(f" NEW ITEM COLLECTED ")
                        self.get_logger().info(f"ITEM X: {self.mapx:.2f} ")
                        self.get_logger().info(f"ITEM Y: {self.mapy:.2f} ")
                        self.get_logger().info(f"ITEM DfH: {self.distance_to_odom:.2f} ")

                        self.previous_state = State.ONWARD
                        self.state = State.SET_GOAL
                        return


                    msg = Twist()
                    msg.linear.x = 0.15
                    self.cmd_vel_publisher.publish(msg)

                    difference_x = self.pose.position.x - self.previous_pose.position.x
                    self.get_logger().info(f"ONWARD DISTANCE: {difference_x:.2f} ")
                    self.get_logger().info(f"TIME SPENT: {self.counter:.2f} ")

                    seconds = 3
                    self.get_logger().info(f"ONWARD {difference_x:.2f} ")
                    self.get_logger().info(f"TIME {self.counter:.2f} ")

                    if self.counter > seconds * (1 / self.timer_period):
                        self.previous_state = State.ONWARD
                        self.state = State.SET_GOAL
                        self.counter = 0
                        return

                    self.counter += 1

                else:

                    self.get_logger().info(f" GET OUT OF ONWARD STATE ")
                    self.previous_state = State.ONWARD
                    self.state = State.SET_GOAL
                    return



            case State.RETURNING:

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map' #ODOM
                goal_pose.header.stamp = self.get_clock().now().to_msg()                
                goal_pose.pose.position.x = -3.3

                goal_pose.pose.position.y = self.returny

                (goal_pose.pose.orientation.x,
                 goal_pose.pose.orientation.y,
                 goal_pose.pose.orientation.z,
                 goal_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(2), axes='sxyz') 

                self.get_logger().info(f"Navigating to: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}), {0:.2f} degrees")

                self.navigator.goToPose(goal_pose) 
                self.previous_state = State.RETURNING
                self.state = State.NAVIGATING
                return 
            

            case State.WAITFORSWAP:
                if self.itemholder.item_value == 15: 
                        self.get_logger().info(f" BLUE ITEM RECOLLECTED ")

                        self.get_logger().info(f"ITEM X: {self.mapx:.2f} ")
                        self.get_logger().info(f"ITEM Y: {self.mapy:.2f} ")
                        self.get_logger().info(f"ITEM DfH: {self.distance_to_odom:.2f} ")

                        self.state = State.RETURNING
                        return
 
                #nudge motion
                msg = Twist()
                msg.angular.x = 0.5
                self.cmd_vel_publisher.publish(msg)
                msg = Twist()
                msg.linear.x = 0.01
                self.cmd_vel_publisher.publish(msg)
                msg = Twist()
                msg.angular.x = -0.5
                self.cmd_vel_publisher.publish(msg)
            
            
                      

            case State.NAVIGATING:

                if self.previous_state != State.GO_TO_WAYPOINT:
                    if (self.itemholder.holding_item and self.itemholder.item_value != 15):
                        self.get_logger().info(f"Item dropped... cancelling navigation")
                        self.navigator.cancelTask()
                        self.state = State.WAITFORSWAP

                        return

                
                if self.previous_state == State.GO_TO_WAYPOINT and self.itemholder.item_value == 15:
                    self.navigator.cancelTask()
                    self.state = State.RETURNING
                    return
                

                if not self.navigator.isTaskComplete():

                    feedback = self.navigator.getFeedback()
                    self.get_logger().info(f"Estimated time of arrival: {(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9):.0f} seconds")

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 30):
                        self.get_logger().info(f"Navigation took too long... cancelling")
                        self.navigator.cancelTask()
                        
                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Goal succeeded!")


                            if self.previous_state == State.GO_TO_WAYPOINT:
                                self.state = State.COLLECTING

                            else:
                                self.previous_state = State.NAVIGATING
                                self.state = State.SET_GOAL
                            
                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Goal was canceled!")
                            
                            self.state = State.FORWARD 


                        case TaskResult.FAILED:
                            self.get_logger().info(f"Goal failed!")

                            self.state = State. FORWARD 

                        case _:
                            self.get_logger().info(f"Goal has an invalid return status!")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
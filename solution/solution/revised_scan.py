import sys

import math
import random
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

import math
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList, ItemHolder, ItemHolders
from solution_interfaces.msg import IsNavigating

def diameter_to_scan(diameter):
    scan_length = (69.0 * float(diameter) ** -0.89) 

    return scan_length

def inf_to_num(x):
    if math.isinf(x):
        return 0.0
    else:
        return x

ITEM_SCAN_THRESHOLD = 50

class RevisedScan(Node):

    def __init__(self):
        super().__init__('revised_scan')

        #-----------------------------------------BASE INITIALIZATION-----------------------------------------------------------------------------------------------

        # Class variables used to store persistent values between executions of callbacks and control loop

        self.items = ItemList()
        self.itemholders = ItemHolder()
        self.scan = LaserScan()
        self.id = self.get_namespace() 
        self.is_navigating = IsNavigating()

        #-------------------------------------------------------------------SUBSCRIBERS----------------------------------------------------------------------------------------------------------



        #SCAN SUBSCRIBER
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSPresetProfiles.SENSOR_DATA.value)
        #ITEM SUBSCRIBER
        self.item_subscriber = self.create_subscription(ItemList, 'items', self.items_callback, 10)
        #ITEMHOLDER SUBSCRIBER
        self.itemholder_subscriber = self.create_subscription(ItemHolders, '/item_holders', self.itemholder_callback, 10)
        #IS NAV SUBSCRIBER
        self.isnavigating_subscriber = self.create_subscription(IsNavigating, 'is_navigating', self.isnavigating_callback, 10)
        

        #-------------------------------------------------------------------------PUBLISHERS-----------------------------------------------------------------------------------------------

        #self.revised_scan_publisher = self.create_publisher(LaserScan, 'revised_scan', QoSPresetProfiles.SENSOR_DATA.value)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', QoSPresetProfiles.SENSOR_DATA.value)

        # Creates a timer that calls the control_loop method repeatedly - each loop represents single iteration of the FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.previous_timer = self.get_clock().now() #


    

    #-------------------------------------------------------------------------------CALLBACK FUNCTIONS------------------------------------------------------------------------------------
    
    def items_callback(self, msg):
        self.items.data = [i for i in msg.data if i.diameter > ITEM_SCAN_THRESHOLD]

    def itemholder_callback(self, msg): #have to loop and find / ordering different everytime
        sorted_itemholders = ItemHolders()
        sorted_itemholderlist = sorted(msg.data, key=lambda x: int(x.robot_id.replace("robot", "")))
        sorted_itemholders.data = sorted_itemholderlist



    #----------------------------------------------------------------SOLUTION TOPIC CALLBACKS-------------------------------------------------------------------------------------

    # Called every time scan_subscriber recieves a LaserScan message from the /scan topic
    #publish revised scan here:
    def scan_callback(self, msg):

        self.scan = msg


    def isnavigating_callback(self, msg):
        self.is_navigating = msg

    def control_loop(self):

        #TRANSLATE IMAGE DATA INTO MODIFIED SCAN DATA FOR SENSOR FUSION:
        if self.is_navigating.isnavigating:
            if len([item for item in self.items.data if item.x >= 0 and item.value < 15]) > 0:
                for i in range(len(self.scan.ranges[0:40])):
                    for item in [item for item in self.items.data if item.x >= 0 and item.value < 15]:
                        if round(math.degrees(item.x/380)) == i:
                            self.scan.ranges[i] = inf_to_num(self.scan.ranges[i])
                            self.scan.ranges[i] = diameter_to_scan(item.diameter) 


            
            if len([item for item in self.items.data if item.x < 0 and item.value < 15]) > 0:
                for i in range(len(self.scan.ranges[312:359])):
                    for item in [item for item in self.items.data if item.x < 0 and item.value < 15]:

                        if round(math.degrees(item.x/380)) == i: 
                            self.scan.ranges[i] = inf_to_num(self.scan.ranges[i])
                            self.scan.ranges[i] = diameter_to_scan(item.diameter)

            
            
            self.scan_publisher.publish(self.scan)
            self.get_logger().info(f"PUBLISHED")
            


    #---------------------------------------------------------------------BASE CALLBACKS----------------------------------------------------------------------------------------------------

    
    

        

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RevisedScan()

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
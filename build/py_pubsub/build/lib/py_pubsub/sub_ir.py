'''
Subscriber example - subscriber to the 7 IR sensors topic
'''
import rclpy
import sys

from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data # qos - quality of service, helps node know info will be using
from irobot_create_msgs.msg import IrIntensityVector

namespace = '' # global variable - particularly useful if using multiple robots

class IRSubscriber(Node):
    
    def __init__(self):
        super().__init__("IR_subscriber") # calls node class constructor, giving it node name IR_subscriber
        print("Creating subscription to the IrIntensityVector type over the /ir_intensity topic")
        # (msg_type, namespace and topic subscribing to, when want to subscribe to it (on a listener callback), type of quality service data receiving)
        self.subscription = self.create_subscription(
            IrIntensityVector, namespace + "/ir_intensity", self.listener_callback, 
            qos_profile_sensor_data) # create subscription
    
    def printIR(self, msg):
        """
        Determines which parts of info computer receives are worth showing
        Want IR values of relevance to us.
        :type msg: IrIntensity
        :rtype: None
        """ 
        print("Printing IR sensor readings: ")
        for reading in msg.readings:
            val = reading.value
            print("IR sensor: " + str(val))
    
    def listener_callback(self, msg:IrIntensityVector):
        """
        Subscriber callback listens and as soon as receives message it runs with help of the printIR function
        """
        print("Now listening to IR sensor readings...")
        self.printIR(msg)

def main(args=None):
    rclpy.init(args=args) # necessary to initialise library in every main function written
    IR_subscriber = IRSubscriber()
    print("Callbacks are called.")
    try:
        rclpy.spin(IR_subscriber)
    except KeyboardInterrupt:
        print("\nCaught keyboard interrupt")
    finally:
        print("Done")
        IR_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
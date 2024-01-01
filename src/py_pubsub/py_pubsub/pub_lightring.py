'''
pub_lightring.py
'''
import sys
import rclpy
from rclpy.node import Node

# The lighring msg is made up of LedColor msgs so both needed
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import LedColor

namespace = '' # currently robot has no namespace assigned

class LEDPublisher(Node):

    def __init__(self):
        super().__init__('led_publisher')

        # Publish lightringleds message over the topic cmd_lightring
        # queue size - 10: queue size is a quality of service (qos) setting limiting queue size, like for subscriber when specificying type of data received
        self.lights_publisher = self.create_publisher(LightringLeds, namespace + '/cmd_lightring', 10)
        self.lightring = LightringLeds()
        self.lightring.override_system = True # allows overriding of the light system, required in the lightringleds message as well as the ledcolors

        timer_period = 2 # how many seconds between consecutive messages published
        self.timer = self.create_timer(timer_period, self.timer_callback)

    
    def timer_callback(self, r=0, g=0, b=255):
        """Control loop publishing the control command to the LED lightring

        Args:
            r (int, optional): LED red channel value. Defaults to 0.
            g (int, optional): LED green channel value. Defaults to 0.
            b (int, optional): LED blue channel value. Defaults to 0.
        """
        print(f"Changing light values to: r={r}, g={g}, b={b}")
        self.lightring.leds= [LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b),
                              LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), 
                              LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b)]
        self.lights_publisher.publish(self.lightring) # publish the message
    
    def reset(self):
        """
        Return control of the lightring back to the robot and reset all leds to white
        """
        white = [LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255),
                 LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255)]
        self.lightring.leds = white
        self.lightring.override_system = False
        self.lights_publisher.publish(self.lightring)
    
def main(args=None):
    rclpy.init(args=args)
    led_publisher = LEDPublisher()

    try:
        rclpy.spin(led_publisher) # spin nodes so callbacks (in this case just the timer callback) called
    except KeyboardInterrupt:
        print("\nCaught Keyboard interrupt")
    finally:
        print("Done")
        led_publisher.reset()
        led_publisher.destroy_node()
        print("Shutting down")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
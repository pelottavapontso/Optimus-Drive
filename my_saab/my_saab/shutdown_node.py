import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import os
import subprocess


def joy_callback(msg):
   
    

    
    
    share_button = msg.buttons[6]
    options_button = msg.buttons[7]

    if share_button == 1 and options_button == 1:
        subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
    





def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('Joy_controller')
    subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_callback(msg), 10)
    

    rclpy.spin(node)

   
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
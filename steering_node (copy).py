
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from sensor_msgs.msg import Joy





    
def joy_callback(msg, publisher_1):
   
    ret_1 = Int32()
    ret_2 = Int32()
    #ret_1.data = msg.buttons[0]
    #ret_2.data = msg.buttons[1]

    if msg.axes[5] == -1 and msg.axes[2] == 1:
    	ret_1.data = 1
    elif msg.axes[5] == 1 and msg.axes[2] == -1:
    	ret_1.data = -1	
    else:
        ret_1.data = 0
    	
    publisher_1.publish(ret_1)
    #publisher_2.publish(ret_2)
    
    
    

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('Joy_controller')
    subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_callback(msg, publisher_1), 10)
    publisher_1 = node.create_publisher(Int32, 'micro_ros_arduino_subscriber', 10)
    #publisher_2 = node.create_publisher(Int32, 'micro_ros_arduino_subscriber_2', 10)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

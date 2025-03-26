
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32

from sensor_msgs.msg import Joy







    
def joy_callback(msg, publisher_1):
   
    ret_1 = Float32()
  

    steering_axes = msg.axes[0]
    triangle_button = msg.buttons[3]

    
    if triangle_button == 1 and steering_axes == 0: 
        ret_1.data = 10.0

    elif steering_axes > 0 and triangle_button == 0:
    	ret_1.data = steering_axes
    elif steering_axes < 0 and triangle_button == 0:
    	ret_1.data = steering_axes
    else:
        ret_1.data = 0.0
        

    	
    publisher_1.publish(ret_1)
 
    
    
    

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('Joy_controller')
    subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_callback(msg, publisher_1), 10)
 
    publisher_1 = node.create_publisher(Float32, 'micro_ros_steering', 10)


    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

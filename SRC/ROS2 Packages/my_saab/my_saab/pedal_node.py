
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32

from sensor_msgs.msg import Joy





    
def joy_callback(msg, publisher_1):
   
    ret_1 = Float32()
    #ret_2 = Int32()
    #ret_1.data = msg.buttons[0]
    #ret_2.data = msg.buttons[1]

    gas_axes = msg.axes[5]
    brake_axes = msg.axes[2]
    
    circle_button = msg.buttons[1]
    x_button = msg.buttons[0]
    gas_start = msg.buttons[5]
    brake_start = msg.buttons[4]


    if circle_button == 1 and gas_axes == 1 and brake_axes == 1 and x_button == 0 and gas_start == 0 and brake_start == 0:
        ret_1.data = 10.0

    elif x_button == 1 and circle_button == 0 and gas_axes == 1 and brake_axes == 1 and gas_start == 0 and brake_start == 0:
        ret_1.data = -10.0
    elif gas_axes < 1 and brake_axes == 1 and x_button == 0 and circle_button == 0 and gas_start == 0 and brake_start == 0:
        ret_1.data = abs(gas_axes-2)-1
    	
    elif gas_axes == 1 and brake_axes < 1 and x_button == 0 and circle_button == 0 and gas_start == 0 and brake_start == 0:
        ret_1.data = -1*(abs(brake_axes-2)-1)
    
    elif circle_button == 0 and gas_axes == 1 and brake_axes == 1 and x_button == 0 and gas_start == 1 and brake_start == 0:
        ret_1.data = 20.0

    elif circle_button == 0 and gas_axes == 1 and brake_axes == 1 and x_button == 0 and gas_start == 0 and brake_start == 1:
        ret_1.data = -20.0

    elif circle_button == 0 and gas_axes == 1 and brake_axes == 1 and x_button == 0 and gas_start == 1 and brake_start == 1:
        ret_1.data = 30.0

    else:
        ret_1.data = 0.0
    	
    publisher_1.publish(ret_1)

    
    
    

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('Joy_controller')
    subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_callback(msg, publisher_1), 10)
    publisher_1 = node.create_publisher(Float32, 'micro_ros_pedal', 10)


    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

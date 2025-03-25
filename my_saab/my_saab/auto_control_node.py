import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import time

class DualTopicSubscriber(Node):
    def __init__(self):
        super().__init__('dual_topic_subscriber')

        # Create a publisher for the output
        self.publisher_1 = self.create_publisher(Float32, 'micro_ros_steering', 10)

        # Create subscriptions for the two topics
        self.subscription_1 = self.create_subscription(
            Float32,
            '/person_detect_distance',
            self.distance_callback,
            10)

        self.subscription_2 = self.create_subscription(
            Float32,
            '/person_detect_frame_width',
            self.width_callback,
            10)

        # Create a subscription for the Joy topic (to detect controller activity)
        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',  # Assuming the Joy topic is '/joy'
            self.joy_callback,
            10)

        # Initialize variables to store the most recent values from each topic
        self.distance_x = None
        self.frame_width = None
        self.controller_active = False  # To track if the controller is active
        self.last_joy_update_time = time.time()  # Timestamp of the last joystick input
        self.time_limit = 3  # Time in seconds to wait before assuming joystick is disconnected

        # Create a timer to periodically check the joystick status
        self.timer = self.create_timer(1.0, self.check_joy_status)  # Check every 1 second

    def distance_callback(self, msg):
        # Store the most recent distance_x value
        self.distance_x = msg.data
        # If both values are available, compute the formula and publish the result
        if self.distance_x is not None and self.frame_width is not None:
            self.calculate_and_publish()

    def width_callback(self, msg):
        # Store the most recent frame_width value
        self.frame_width = msg.data
        # If both values are available, compute the formula and publish the result
        if self.distance_x is not None and self.frame_width is not None:
            self.calculate_and_publish()

    def joy_callback(self, msg):
        # Update the last time joystick input was received
        self.last_joy_update_time = time.time()
        
        # Check if the controller is active (any button or axis has non-zero value)
        if any(axis != 0 for axis in msg.axes) or any(button != 0 for button in msg.buttons):
            self.controller_active = True  # Controller is active
        else:
            self.controller_active = False  # Controller is inactive

    def check_joy_status(self):
        # Check if joystick has been inactive for more than time_limit seconds
        current_time = time.time()
        if self.controller_active and current_time - self.last_joy_update_time > self.time_limit:
            self.controller_active = False

    def calculate_and_publish(self):
        # If the controller is active, stop publishing
        if self.controller_active:
            return  # Don't publish anything if the controller is active

        # Compute the formula: msg.data = -1 * distance_x / (frame_width / 4)
        result = -1 * self.distance_x / (self.frame_width / 2)

        # Create the message to be published
        result_msg = Float32()
        result_msg.data = result

        # Publish the result
        self.publisher_1.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)

    node = DualTopicSubscriber()

    # Spin the node to keep it alive and handle incoming messages
    rclpy.spin(node)

    # Explicitly destroy the node after spinning
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

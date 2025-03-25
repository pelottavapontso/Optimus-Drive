import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensor_msgs.msg import Joy

import torch
import cv2
import numpy as np


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_distance = self.create_publisher(Float32, '/person_detect_distance', 10)
        self.publisher_frame_width = self.create_publisher(Float32, '/person_detect_frame_width', 10)

        self.frame_width = 1280
        self.frame_height = 720

        self.msg_distance = Float32()
        self.msg_frame_width = Float32()
        self.msg_frame_width.data = float(self.frame_width)

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model = self.model.to("cuda" if torch.cuda.is_available() else "cpu")  # Use GPU if available
        self.model.eval()
        
        timer_period = 0.5  #second
        self.timer = self.create_timer(timer_period, self.timer_callback)
      

    def timer_callback(self):
               

        # Open webcam (0 = default camera, change to 1, 2 if using multiple cameras)
        cap = cv2.VideoCapture(0)

        # Set webcam resolution (optional)       
        cap.set(3, self.frame_width)  # Width
        cap.set(4, self.frame_height)  # Height

        screen_center_x = self.frame_width // 2

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("❌ Failed to grab frame.")
                break

            # Convert frame to RGB (YOLOv5 expects RGB images)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Run YOLOv5 inference
            results = self.model(frame_rgb)

            # Get detection results as a pandas DataFrame
            detections = results.pandas().xyxy[0]

            distance_x = 0.0

            # Draw bounding boxes and labels
            for _, row in detections.iterrows():
                if row['name'] == 'person':         #Only detect humans
                    x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                    confidence = row['confidence']
                    label = f"{row['name']} {confidence:.2f}"

                    # Calculate the object's center (X-axis)
                    obj_center_x = (x1 + x2) // 2

                    # Calculate distance in X-axis from screen center
                    distance_x = obj_center_x - screen_center_x
                    
                    
                    

                    #print(distance_x)

                    
                    # msg.data = -1*distance_x/(frame_width/4)       #steering signal in range -2...2 to esp32

                
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            
                    
            self.msg_distance.data = float(distance_x)
                    

            
            

            # Show the live feed with detections
            cv2.imshow("Live Object Detection", frame)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            self.publisher_distance.publish(self.msg_distance)
            #self.get_logger().info('Publishing: "%s"' % msg_distance.data)

            self.publisher_frame_width.publish(self.msg_frame_width)
            #self.get_logger().info('Publishing: "%s"' % msg_frame_width.data)


           

        # Release resources
        cap.release()
        cv2.destroyAllWindows()







        




def main(args=None):
    rclpy.init(args=args)

    
    
    publisher = Publisher()
    rclpy.spin(publisher)

    

    

    

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
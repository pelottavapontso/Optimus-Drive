# Optimus Drive

Project idea was to create a drive by wire kit for old automatic transmission cars.
This GitHub repository presents all the codes used in the project.
The project was carried out in cooperation with Aalto University.


![Saab](DOC/gif.mp4)



This repostory includes following folders:
- DOC folder that contains instructions and project document files
- SRC folder that contains ROS2 package with nodes in folder SRC/ROS2 packages/ and ESP32 codes in folder SRC/ESP32 codes/



## Table of Contents
- [Description](#description)
- [Installation](#installation)
- [License](#license)

## Description

The project was done with old saab 900 automatic transmission car.
Hardwares used in this project were Nvidia Jetson Orion Nano and ESP32.
Jetson Orion Nano was useb with Ubuntu and ROS2.


## Installation

To install and run this project locally, follow the steps below:

ROS2 package:

1. Download ROS2 to your Ubuntu computer
2. Create workspace with src folder
3. Copy my_saab ROS2 package from this GitHub repositorys SRC/ROS2 packages/ to your workspaces src folder
4. Build workspace

All following steps are Ubuntu terminal commands to install micro-ROS-agent:

5. source /opt/ros/humble/setup.bash
6. cd <to your workspace>
7. git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
8. cd ..
9. rosdep install --from-paths src --ignore-src -r –y
10. cd <to your workspace>
11. colcon build
12. ros2 run micro_ros_agent micro_ros_agent serial –dev /dev/ttyUSB0


ESP32 library and code:

1. Install Arduino IDE
2. Go to file/properties/ and paste following link to "Additional Boards Manager URL:s": https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
3. Install esp32 core by Espressif Systems from arduino ide tools/board manager
4. Choose esp32 dev module board from tools/board manager
5. Install micro_ros_arduino library to Arduino IDE by following steps
6. Go to following github and download the code to ZIP file:    https://github.com/micro-ROS/micro_ros_arduino/tree/humble
7. Arduino ide press sketch/library manager/ and click from .ZIP file and choose the downloaded ZIP file
8. Download ESP32 codes from this GitHub repostory SRC/ESP32 codes
9. Send the code to your ESP32

More instructions in this GitHub repostory DOC/


## License

Following GitHub repostories used in this project:
- https://github.com/ultralytics/yolov5
- https://github.com/micro-ROS/micro_ros_arduino
- https://github.com/micro-ROS/micro-ROS-Agent

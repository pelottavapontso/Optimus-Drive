
Instructions to install micro-ROS to ESP32:

1. Install Arduino IDE
2. Go to file/properties/ and paste following link to "Additional Boards Manager URL:s": https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
3. Install esp32 core by Espressif Systems from arduino ide tools/board manager
4. Choose esp32 dev module board from tools/board manager
5. Install micro_ros_arduino library to Arduino IDE by following steps
6. Go to following github and download the code to ZIP file:    https://github.com/micro-ROS/micro_ros_arduino/tree/humble
7. Arduino ide press sketch/library manager/ and click from .ZIP file and choose the downloaded ZIP file

Now you have micro_ros library on your esp32
More detailed instustions on following youtube video: https://www.youtube.com/watch?v=qtVFsgTG3AA



Instructions to install micro-ROS-agent to ROS2:


All the steps are Ubuntu terminal commands.

1. source /opt/ros/humble/setup.bash
2. cd <to your workspace>
3. git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
4. cd ..
5. rosdep install --from-paths src --ignore-src -r –y
6. cd <to your workspace>
7. colcon build
8. ros2 run micro_ros_agent micro_ros_agent serial –dev /dev/ttyUSB0

Now you have micro-ROS-agent on your ROS2
More detailed instustions on following youtube video: https://www.youtube.com/watch?v=qtVFsgTG3AA


To get the Jetson Orion Nano to autolaunch ROS2 nodes when it start needs to put following crontab command.

Following steps to Ubuntu terminal:

1. crontab -e
2. @reboot bash -c "sleep 30 && export DISPLAY=:0 && source /opt/ros/humble/setup.bash && source /home/amlab/optimus_drive/install/setup.bash && ros2 launch my_saab launch.py > /home/ amlab/ros2_launch.log 2.log 2>&1"
3. control + x and save
# Optimus-Drive

@reboot bash -c "sleep 30 && export DISPLAY=:0 && source /opt/ros/humble/setup.bash && source /home/amlab/optimus_drive/install/setup.bash && ros2 launch my_saab launch.py > /home/amlab/ros2_launch.log 2.log 2>&1"

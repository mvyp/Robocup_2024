gnome-terminal -t "base" -x bash -c "roslaunch mrobot_bringup res.launch;exec bash;"
sleep 1s
#gnome-terminal -t "robot" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"
#sleep 1s
#gnome-terminal -t "Azure_Kinect_Driver" -x bash -c "roslaunch /home/msi/robocup_ws/src/Azure_Kinect_ROS_Driver/launch/driver.launch;exec bash;"
#sleep 1s
#gnome-terminal -t "moveit" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch; exec bash;"
#gnome-terminal -t "pan_tilt_bringup" -x bash -c "roslaunch /home/msi/robocup_ws/src/driver/pan_tilt_ros/pan_tilt_bringup/launch/panTilt_bringup.launch; exec bash;"
#sleep 1s
gnome-terminal -t "pan" -x bash -c "rosrun pan_tilt_driver run_pan.py;exec bash;"
#sleep 2s
#gnome-terminal -t "yolo_v5_bringup" -x bash -c "roslaunch /home/msi/kinova_ws/src/Yolov5_ros/yolov5_ros/yolov5_ros/launch/yolo_v5.launch; exec bash;"

#gnome-terminal -t "image_converter" -x bash -c "python3 '/home/msi/teamwork/image_converter/src/image_converter_node.py'; exec bash;"

#gnome-terminal -t "find_object_2d" -x bash -c "rosrun find_object_2d find_object_2d image:=/rgb/image_color; exec bash;"

#sleep 2s
#gnome-terminal -t "tidy_up" -x bash -c "cd ~/Robocup_teamwork; python3 tidy_up.py;exec bash;"

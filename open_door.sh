
# gnome-terminal -t "robot" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"
# sleep 3
# gnome-terminal -t "camera" -x bash -c "roslaunch realsense2_camera rs_d435_camera_with_model.launch;exec bash"
# gnome-terminal -t "camera" -x bash -c "roslaunch realsense2_camera rs_camera.launch ;exec bash"
# gnome-terminal -t "camera" -x bash -c "roslaunch azure_kinect_ros_driver hand_eye.launch;exec bash"

gnome-terminal -t "base" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"


sleep 2s
gnome-terminal -t "moveit" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch; exec bash;"
sleep 2s
#gnome-terminal -t "open_door" -x bash -c "rosrun kinova_arm_moveit_demo kinova_open_door.py ;exec bash;"


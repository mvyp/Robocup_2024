

gnome-terminal -t "mrobot_bringup" -x bash -c "roslaunch mrobot_bringup res.launch;exec bash;"
sleep 5s
gnome-terminal -t "robot" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"
sleep 1s
#gnome-terminal -t "Receptionist" -x bash -c "cd ~/Robocup_teamwork; python3 Receptionist.py ;"
gnome-terminal -t "moveit" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch; exec bash;"
sleep 2s
gnome-terminal -t "yolov5_ros" -x bash -c "conda activate pytorch ;roslaunch yolov5_ros yolo_v5.launch ;"





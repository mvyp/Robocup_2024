gnome-terminal -t "what_is_that" -x bash -c "source /opt/ros/melodic/setup.bash; source ~/robocup_ws/devel/setup.bash; roslaunch '/home/msi/robocup_ws/src/mrobot/mrobot_bringup/launch/what_is_that.launch';"

#gnome-terminal -t "pytorch" -- bash -c "conda activate pytorch; python3 '/home/msi/robocup_ws/src/third_party/hand_reco/what_is_that.py';"


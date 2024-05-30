# Robocup_2024
# teamwork
# Video 
https://www.bilibili.com/video/BV1Yw4m1S7pn/?share_source=copy_web&vd_source=1d11b082e214e205bf1127f07df592b2
# HARROLD_xjw Equipment

Kinova robotic arm            Kinect Arzue depth camera        
# tidu_up.sh:
The robot can automatically navigate to the desktop to clean up garbage in the home environment: 

YOLOV5 identify the type of object, grab it and throw it to the trash can or put it back

Use the point cloud, calculate the depth information of bounding_box pixels to process the average, and get the position coordinates of the center of gravity point of the object relative to the camera, 
publish its tf

The basic functions of navigation goto (goal) and speech are defined

The robot human is defined so that navigation, robotic arm visual grasping and speech functions can be integrated

# vision:
gpd_ros 

object_position.py

object_tf.cpp

Yolov5_ros yolov5.launch


navigation and slam:
res.launch

slam.launch

navigation.launch

<img width="1409" alt="6a843e9f30220092e7f8bbf770665b3" src="https://github.com/mvyp/Robocup_2024/assets/142517129/6b77ddca-f442-42da-8e71-9b8217ba3551">


GPSR: Difficult
follwer

take the object to placement2

find the object which we need

person how many |gender|

# what is that:
what is that.sh
The second line needs to start conda pytorch separately

The right hand is placed on the left shoulder to activate the follower

Raising your arm upwards activates the recognition function, which then points to the object you want it to broadcast.

The robot calculates the point of your finger, follows you and broadcasts the type of object you are pointing to and features such as color

# Rceptionist:
# Open door.sh
The task flow of entertaining guests:

1. The robot arm opens the door


2. The guest will automatically navigate to the designated location, introduce the guest's name and favorite drink to the host, and guide the guest to an empty seat


mian class init

navigation--self.goto(0,0,0)

# Speech to text
Azure subscriptions expire
The rviz node of the camera --notic same node name
Multithreading calls do not interrupt _thread.start_new_thread(text_to_speech, ("task begin!" ))
You can also just call text_to_speech("hurry").

Text-to-speech speech_to_text returns text

TF tree view error --rosrun rqt_tf_tree rqt_tf_tree

Node versus topic check --rosrun rqt_graph rqt_graph

# position and orientation:
Save the map:
rosrun map_server map_saver -f my2024_map
python3 '/home/xxx/scripts/teleop_twist_keyboard/teleop_twist_keyboard.py'

The whole point of github is to be able to look at commits and see what's different

# Thanks to all the members of the 2024 BUCT Robotics Center

# Thank senior schoolmates for vital help 

![1716911297515](https://github.com/mvyp/Robocup_2024/assets/142517129/0aa21fcb-5cd7-4673-a471-d5cd2881c13a)
![1716911339113](https://github.com/mvyp/Robocup_2024/assets/142517129/4b1e5fb0-7ef3-4f19-82b5-cc53b5758d2f)


![eb2bfe97192feb813110e6f5cb01d17](https://github.com/mvyp/Robocup_2024/assets/142517129/2c215d19-3414-4eeb-86c5-33a0da105222)


用github的目的是可以查看提交的修改记录，找到每次不同
感谢2024机器人中心所有成员的努力
感谢往届学长的至关重要的帮助

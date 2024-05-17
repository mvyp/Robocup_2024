# Robocup_2024
# teamwork
HARROLD_xjw
Kinova robotic arm kinect     Arzue depth camera     McNamullen   usb485
# tidu_up.sh:
The robot can automatically navigate to the desktop to clean up garbage in the home environment: identify the type of object, grab it and throw it to the trash can or put it back
use the point cloud, calculate the depth information of bounding_box pixels to process the average, and get the position coordinates of the center of gravity point of the object relative to the camera, 
and publish it through tf
The basic functions of navigation goto (goal) and speech are defined
The robot human is defined so that navigation, robotic arm visual grasping and speech functions can be integrated

# comupter vision:
gpd_ros object_position.py
Yolov5_ros yolov5.launch


navigation and slam:
res.launch
slam.launch
navigation.launch
<img width="1409" alt="6a843e9f30220092e7f8bbf770665b3" src="https://github.com/mvyp/Robocup_2024/assets/142517129/6b77ddca-f442-42da-8e71-9b8217ba3551">

GPSR: Difficult
follwer# own operator
take the object to placement2
find the object which we need
person how many |gender|--unfinishd

# what is that:
what is that.sh
The second line needs to start conda pytorch separately
The right hand is placed on the left shoulder to activate the follower
Raising your arm upwards activates the recognition function, which then points to the object you want it to broadcast.
The robot calculates the point of your finger, follows you and broadcasts the type of object you are pointing to and features such as color

# Rceptionist:
The process of robot entertaining guests is realized
It can realize the autonomous door opening voice to ask the guest to like the drink and guide the guest to sit on the empty sofa
The main function in class is the task flow


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

Debug position and orientation:
Save the map:
rosrun map_server map_saver -f my2024_map
python3 '/home/xxx/scripts/teleop_twist_keyboard/teleop_twist_keyboard.py'

The whole point of github is to be able to look at commits and see what's different

# Thanks to all the members of the 2024 BUCT Robotics Center

# Thank you for your vital help in previous years


# Robocup_2024
# teamwork
HARROLD_xjw
Kinova 机械臂 kinect Arzue深度相机 

tidu_up.sh：
实现了机器人在家庭环境中，自动导航至桌面清理垃圾的流程：识别物体类型 抓取并扔到垃圾桶或放回原处
定义了导航和语音的基本函数
定义了机器人类，从而集成实现导航、机械臂视觉抓取和语音功能

  comupter vision:
    gpd_ros object_position.py
    Yolov5_ros yolov5.launch


  navigation and slam:
    res.launch
    slam.launch
    navigation.launch

GPSR:困难
follwer#自己的操作员
take the object to placement2
find the object which we need
person how many |gender|--unfinishd



what is that：
what is that.sh
第二行需要单独启动conda pytorch
右手放在左肩膀可启动follower
向上高举胳膊可以启动识别功能，然后就可以指向你想让他播报的objects，机器人会计算你的手指指向，跟随你并且播报你所指向的物品种类和例如颜色等特征

Rceptionist:
实现了机器人招待客人的过程
class中main函数为 任务流程
<img width="1409" alt="6a843e9f30220092e7f8bbf770665b3" src="https://github.com/mvyp/Robocup_2024/assets/142517129/e9834a4e-badc-4c61-a8d5-bc5bec212998">


主要看 mian 及 类的init

导航去某一个点 self.goto(0,0,0)---功能完善

语音转文字 
Azure的订阅会过期
相机的rviz节点命名
多线程调用不会中断    _thread.start_new_thread( text_to_speech, ("task begin!",) )
也可直接调用 text_to_speech("hurry")

文字转语音  speech_to_text  会return text

TF树查看错误 rosrun rqt_tf_tree rqt_tf_tree
节点与话题检查 rosrun rqt_graph rqt_graph

调试位置与朝向：
<!-- ============== Rviz MAP position ==============-->
<node if="$(arg rviz)" pkg="rviz" type="rviz" name="rviz" required="true"
args="-d $(find mrobot_navigation)/rviz/mrobot_nav.rviz"/>
保存地图：
rosrun map_server map_saver -f my2024_map
python3 '/home/msi/robocup_ws/src/mrobot/mrobot_bringup/scripts/teleop_twist_keyboard/teleop_twist_keyboard.py' 

用github的目的是可以查看提交的修改记录，找到每次不同

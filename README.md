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

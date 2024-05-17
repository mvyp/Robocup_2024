print('loading model')
import rospy
import sys
import _thread
from std_msgs.msg import Bool,String
from roslib import message
from geometry_msgs.msg import Twist
from math import copysign
from visualization_msgs.msg import MarkerArray
import azure.cognitiveservices.speech as speechsdk
from sensor_msgs.msg import Image
from pan_tilt_msgs.msg import PanTiltCmdDeg
from cv_bridge import CvBridge, CvBridgeError

from collections import Counter   # 用于统计
import time

#from tool import *
#from yolo_tool import *

#subprocess.call(['python', '-m', 'ensurepip', '--upgrade'])
things = ['Biscuit', 'Bread','Chip','Cola','Cookie','Dishsoap','Handwash','Lays','Orange juice','Shampoo','Sprite','Water']
color_list = [(100, 0, 230), (100, 0, 0), (10,30, 20), (234, 34, 2), (23,4, 244), (24, 0, 23), (200, 100, 0), (250, 0, 0),(100, 0, 230), (100, 0, 0), (10,30, 20), (234, 34, 2), (23,4, 244), (24, 0, 23), (200, 100, 0), (250, 0, 0)]
if len(things) > len(color_list):
    print('请补充颜色个数,目前识别物类别过多')
#     sys.exit()
#predictor = init()

show_name = 'First_test'
duration = 15  # 总持续时间（秒）
interval = 1  # 发布间隔（秒）
end_time = time.time() + duration
global a
a=False


class Follower():
    def __init__(self):
        rospy.init_node("follower")
        #
        self.robotstate= False
        self.trigger=False
        self.time_count=time.time()
        self.bridge = CvBridge()
        self.tmp_point_thing = []   # 储存历史指向数据,用于判断是否播放语音
        self.play_threshold = 10    # 进过 play_threshold 次判断都是同意指向,才进行播报
        self.time_threshold = 10    # 至少间隔  几秒 才可播放一次
        self.temp_t = 0
        self.feature_dict={
            'Biscuit': 'blue with white',
            'Orange juice': "orange",
            'Cola':"red with black",
            'Water':"white with red",
            'Lays':"yellow with green",
            'Bread':"white",
            'Cookie':"black with yellow",
            'Chip':"brown",
            'Shampoo':"blue",
            'Dishsoap':"yellow",
            'Handwash':"pink",
            'Sprite':"green"
        }

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 1)
        self.time_begin=0
        # How far away from the goal distance (in meters) before the robot reacts  0.1
        self.z_threshold = rospy.get_param("~z_threshold", 0.1)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight left/right displacement of the person when making a movement   2.5     
        self.x_scale = rospy.get_param("~x_scale", 1)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.5)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.01)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.03)
        
        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)
        
        # Initialize the movement command
        self.move_cmd = Twist()
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.shutdown_apf_pub = rospy.Publisher('/apf_shutdown', Bool, queue_size=5)
        self.pub = rospy.Publisher('/thing_pointed', String, queue_size=10)
        self.pub_pts = rospy.Publisher('/thing_pointed_pts', String, queue_size=10)
        self.pub_pan_tilt = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=1)
        # Subscribe to the point cloud
        self.marker_subscriber = rospy.Subscriber('/body_tracking_data', MarkerArray, self.set_cmd_vel, queue_size=1)
        self.pan_tilt_down=PanTiltCmdDeg()       
        self.pan_tilt_down.pitch=40.0
        self.pan_tilt_down.speed=20
        self.pan_tilt_up=PanTiltCmdDeg()
        self.pan_tilt_up.pitch=0
        self.pan_tilt_up.speed=20

        rospy.loginfo("Ready to follow!")
        try:
            _thread.start_new_thread( voice, ("Ready to follow! Please put your right hand on your left shoulder or put your left hand on your right shoulder and I will follow you",) )
            
        except:
            print("Error: unable to start thread")
        

        
    def set_cmd_vel(self, msg):
        if(msg!= MarkerArray()):
            # Check start position
            #5: SHOULDER_LEFT 7: WRIST_LEFT 8:HAND_LEFT 12:SHOULDER_RIGHT 14:WRIST_RIGHT 15:HAND_RIGHT 26:HEAD,
            go_2y=abs((msg.markers[15].pose.position.y +msg.markers[14].pose.position.y)/2 - msg.markers[5].pose.position.y)
            go_1y=abs((msg.markers[8].pose.position.y +msg.markers[7].pose.position.y)/2 - msg.markers[12].pose.position.y)
            go_2x=abs((msg.markers[15].pose.position.x +msg.markers[14].pose.position.x)/2 - msg.markers[5].pose.position.x)
            go_1x=abs((msg.markers[8].pose.position.x +msg.markers[7].pose.position.x)/2 - msg.markers[12].pose.position.x)
            stop_left_arm=(msg.markers[8].pose.position.y - msg.markers[26].pose.position.y)
            stop_right_arm=(msg.markers[15].pose.position.y - msg.markers[26].pose.position.y)
            # print(right_arm)
            # print(left_arm)
            #print(stop_1)
            #print(stop_2)
            #print(msg.markers[1].pose.position.x)r
            #print("--------------------------")
            
            if (go_1y<0.07 and go_1x<0.07) or (go_2y<0.07 and go_2x <0.07):
                self.robotstate=True
                self.trigger=True
                now_time=time.time()
                if(now_time- self.time_count >4):
                    self.time_count = time.time()
                    self.pub_pan_tilt.publish(self.pan_tilt_up)
                    voice ("Moving.")
            
            if stop_right_arm<-0.25 or stop_left_arm<-0.25:
                global a
                a=True
                self.tmp_point_thing = []
                self.robotstate=False
                temp = Bool()
                temp.data=True
                self.shutdown_apf_pub.publish(temp)
                now_time=time.time()
                if(now_time- self.time_count >4):
                    self.time_count = time.time()
                    self.pub_pan_tilt.publish(self.pan_tilt_down)
                    voice ("Stopping the robot.")
                

                
            # Initialize the centroid coordinates point count
            x = y = z = n = 0
            
        # If we have points, compute the centroid coordinates
            if self.robotstate and msg.markers[1].pose.position.z<1.5:
                #z distance x leftright y updown
                x = msg.markers[1].pose.position.x 
                z = msg.markers[1].pose.position.z
                
                # Check our movement thresholds
                if (abs(z - self.goal_z) > self.z_threshold):
                    # Compute the angular component of the movement
                    linear_speed = (z - self.goal_z) * self.z_scale
                    
                    # Make sure we meet our min/max specifications
                    self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                            min(self.max_linear_speed, abs(linear_speed))), linear_speed)
                else:
                    self.move_cmd.linear.x *= self.slow_down_factor
                    
                if (abs(x) > self.x_threshold):     
                    # Compute the linear component of the movement
                    angular_speed = -x * self.x_scale
                    
                    # Make sure we meet our min/max specifications
                    self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                            min(self.max_angular_speed, abs(angular_speed))), angular_speed)
                else:
                    # Stop the rotation smoothly
                    self.move_cmd.angular.z *= self.slow_down_factor
                    
            else:
                # Stop the robot smoothly
                self.move_cmd.linear.x *= self.slow_down_factor
                self.move_cmd.angular.z *= self.slow_down_factor
            # Publish the movement command
            self.cmd_vel_pub.publish(self.move_cmd)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Unregister the subscriber to stop cmd_vel publishing
        
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)   





        
def voice (word):
    global speech_synthesizer
    speech_synthesizer.speak_text_async(word).get() 
    print("voice finish")      
                 
if __name__ == '__main__':
    
    speech_key, service_region = "b756dfeefd0f4947a88b5b0b30826943", "eastus"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
    speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
    speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)   
    
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")





# K4ABT_JOINT_PELVIS = 0, 
# K4ABT_JOINT_SPINE_NAVEL,
# K4ABT_JOINT_SPINE_CHEST, 
# K4ABT_JOINT_NECK,
#   K4ABT_JOINT_CLAVICLE_LEFT,
#   K4ABT_JOINT_SHOULDER_LEFT,
#   K4ABT_JOINT_ELBOW_LEFT,
#   K4ABT_JOINT_WRIST_LEFT,
#   K4ABT_JOINT_HAND_LEFT,
#   K4ABT_JOINT_HANDTIP_LEFT, 
#   K4ABT_JOINT_THUMB_LEFT,
#   K4ABT_JOINT_CLAVICLE_RIGHT,
#   K4ABT_JOINT_SHOULDER_RIGHT,
#   K4ABT_JOINT_ELBOW_RIGHT, 
#   K4ABT_JOINT_WRIST_RIGHT,
#   K4ABT_JOINT_HAND_RIGHT,
#   K4ABT_JOINT_HANDTIP_RIGHT,
#   K4ABT_JOINT_THUMB_RIGHT,
#   K4ABT_JOINT_HIP_LEFT,
#   K4ABT_JOINT_KNEE_LEFT,
#   K4ABT_JOINT_ANKLE_LEFT,
#   K4ABT_JOINT_FOOT_LEFT,
#   K4ABT_JOINT_HIP_RIGHT, 
#   K4ABT_JOINT_KNEE_RIGHT,
#   K4ABT_JOINT_ANKLE_RIGHT, 
#   K4ABT_JOINT_FOOT_RIGHT,
#   K4ABT_JOINT_HEAD,
#   K4ABT_JOINT_NOSE,
#   K4ABT_JOINT_EYE_LEFT, 
#   K4ABT_JOINT_EAR_LEFT,
#   K4ABT_JOINT_EYE_RIGHT, 
#   K4ABT_JOINT_EAR_RIGHT,
#   K4ABT_JOINT_COUNT
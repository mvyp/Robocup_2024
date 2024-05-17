#!/usr/bin/env python3
import rospy
import time
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_conversions import transformations
from math import pi
import tf
import _thread
import azure.cognitiveservices.speech as speechsdk
from std_msgs.msg import String
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBoxes
import cv2 as cv
import math
from cv_bridge import CvBridge, CvBridgeError
from pan_tilt_msgs.msg import PanTiltCmdDeg
import dynamic_reconfigure.client


class receptionist:

    def __init__(self):
        
        #subsciber
        # self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.img_callback, queue_size=1)

        #publisher
        self.pub_cmd = rospy.Publisher('Command', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.set_pose_pub = rospy.Publisher('/initialpose',
                                            PoseWithCovarianceStamped,
                                            queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.tf_listener = tf.TransformListener()
        self.pub_pan_tilt = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=1)

        try:
            self.tf_listener.waitForTransform('/map',
                                              '/base_link', rospy.Time(),
                                              rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        self.pan_tilt_down=PanTiltCmdDeg()       
        self.pan_tilt_down.pitch=40.0
        self.pan_tilt_down.speed=20

        #variable TODO
        self.master_name = 'jack'
        self.master_drink = 'orangejuice'
        self.master_cloth = 'unknow'

        self.guest1_name = 'unknow'
        self.guest1_drink = 'unknow'
        self.guest1_upper_cloth = 'unknow'
        self.guest1_lower_cloth = 'unknow'
        self.guest1_age = 'unknow'
        self.guest1_glass = 'unknow'
        self.guest1_sex = 'unknow'


        self.guest2_name = 'unknow'
        self.guest2_drink = 'unknow'
        self.guest2_upper_cloth = 'unknow'
        self.guest2_lower_cloth = 'unknow'
        self.guest2_age = 'unknow'
        self.guest2_glass = 'unknow'
        self.guest2_sex = 'unknow'

        self.move_cmd = Twist()
        self.bridge_ros2cv = CvBridge()
        self.door_position = [-1.022,-0.007,180]
        self.detect_1 = [1.169, 0.156,90]
        self.detect_2 = [2.133, 0.176,90]
        self.detect_3 = [3.857, 0.176,90]
        self.detect_4 = [4.500, 0.156,90]

        self.postion_x =0
        self.postion_y =0
        self.orientation_x =0
        self.orientation_y =0
        self.orientation_z =0
        self.orientation_w =0
        self.client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS/")
            
# ----------Navigation-------------------------------------------------------------------------

    def get_pos(self):
        try:
            (trans,
             rot) = self.tf_listener.lookupTransform('/map', '/base_link',
                                                     rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return False
        euler = transformations.euler_from_quaternion(rot)
        x = trans[0]
        y = trans[1]
        th = euler[2] / pi * 180
        return (x, y, th)

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("goal reached! ")

    def _active_cb(self):
        rospy.loginfo("navigation has be actived")

    def _feedback_cb(self, feedback):
        self.postion_x =feedback.base_position.pose.position.x
        self.postion_y =feedback.base_position.pose.position.y
        self.orientation_x =feedback.base_position.pose.orientation.x
        self.orientation_y =feedback.base_position.pose.orientation.y
        self.orientation_z =feedback.base_position.pose.orientation.z
        self.orientation_w =feedback.base_position.pose.orientation.w

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def goto(self, p):
        global xc, yc, zc
        rospy.loginfo("[Navi] goto %s" % p)
        x, y, th = p
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb,
                                 self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    def empty_seat1_first(self):        

        linear=Twist()
        has_people=0
        try:
            data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            print("no people")
            return 0
        print(data)
        print("---------------")
        try:
            for i in range(data.bounding_boxes[0].num):
                if((  data.bounding_boxes[i].xmax<2800 )and  ( data.bounding_boxes[i].xmin>500)and data.bounding_boxes[i].ymax>800):
                    has_people=1
        except:
            pass
        if (not has_people):
            return 0
                
        has_people=0


####################################################################
        self.goto(self.detect_2)


        try:
            data2 = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            return 0
        print("data2")
        print(data2)
        print("---------------")
        try:
            for i in range(data2.bounding_boxes[0].num):
                if((  data2.bounding_boxes[i].xmax<2700 )and  ( data2.bounding_boxes[i].xmin>500) and data2.bounding_boxes[i].ymax>800):
                    has_people=1
            if (not has_people):
                    return 0
        except:
            pass
        has_people=0


###########################################################
        self.goto(self.detect_3)


        try:
            data3 = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            return 0
        print(data3)
        print("---------------")
        try:
            for i in range(data3.bounding_boxes[0].num):
                if((  data3.bounding_boxes[i].xmax<2500 )and  ( data3.bounding_boxes[i].xmin>500)and data3.bounding_boxes[i].ymax>8000):
                    has_people=1
            if (not has_people):
                    return 0
        except:
            pass
        has_people=0
###########################################################        
        self.goto(self.detect_4)

        return 0



    def master1_hzx(self):
        try_times = 3 # 尝试检测三次
        print("1234567890-345678")
        for i in range(try_times): 
            try:
                data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
                for i in range(data.bounding_boxes[0].num):
                    # print(data)
                    if (data.bounding_boxes[i].xmax < 2800) and (data.bounding_boxes[i].xmin > 400) and (data.bounding_boxes[i].ymax > 500):
                        self.master_posiiton1 = 1 # 第一次接待客人时主人在第一个点
                        _thread.start_new_thread(text_to_speech, ("found master at point one", ))
                        return 0
            except:
                pass
            #_thread.start_new_thread(text_to_speech, ("didn't find master at point one, retrying in 3 seconds", ))
  

        self.goto(self.detect_2)


        try_times = 3 # 尝试检测三次
        for i in range(try_times): 
            try:
                data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
                for i in range(data.bounding_boxes[0].num):
                    if (data.bounding_boxes[i].xmax < 2800) and (data.bounding_boxes[i].xmin > 00) and (data.bounding_boxes[i].ymax > 500):
                        self.master_posiiton1 = 2 # 第一次接待客人时主人在第二个点
                        _thread.start_new_thread(text_to_speech, ("found master at point two", ))

                        return 0
            except:
                pass
            #_thread.start_new_thread(text_to_speech, ("didn't find master at point two, retrying in 3 seconds", ))
      

        self.goto(self.detect_3)

        try_times = 3 # 尝试检测三次
        for i in range(try_times): 
            try:
                data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
                for i in range(data.bounding_boxes[0].num):
                    # print(data)
                    if (data.bounding_boxes[i].xmax < 2800) and (data.bounding_boxes[i].xmin > 400) and (data.bounding_boxes[i].ymax > 500):
                        self.master_posiiton1 = 3 # 第一次接待客人时主人在第三个点
                        _thread.start_new_thread(text_to_speech, ("found master at point three", ))
                        return 0
            except:
                pass
            #_thread.start_new_thread(text_to_speech, ("didn't find master at point one, retrying in 3 seconds", ))
     

        self.goto(self.detect_4)

    #     # if(data.bounding_boxes[0].num==1):#
    #     # if(data.bounding_boxes[0].num==1):
    #     #     print(data.bounding_boxes[0].xmin)
    #     #     print(data.bounding_boxes[0].xmax)
    #     #     print("--------------------")
    #     #     if(data.bounding_boxes[0].xmin>1100 and data.bounding_boxes[0].xmax<2300):
    #     #         angular.angular.z=-0.3
    #     #         self.cmd_vel_pub.publish(angular)
    #     #         rospy.sleep(1)
    #     #         self.cmd_vel_pub.publish(Twist())
    #     # if(data.bounding_boxes[0].num==2):
    #     #     for i in range(2):
    #     #         if(data.bounding_boxes[i].xmin<1000):
    #     #             seat1=0

    #     #         if(data.bounding_boxes[i].xmax>2800):
    #     #             seat3=0
    #     #     if(seat1):
    #     #         angular.angular.z=-0.3
    #     #         self.cmd_vel_pub.publish(angular)
    #     #         rospy.sleep(1)
    #     #         self.cmd_vel_pub.publish(Twist())
    #     #     if(seat3):
    #     #         angular.angular.z=0.3
    #     #         self.cmd_vel_pub.publish(angular)
    #     #         rospy.sleep(1)
    #     #         self.cmd_vel_pub.publish(Twist())
    def second(self):        

        linear=Twist()
        has_people1=0
        who_master= 0
        try:
            data1 = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            print("no people")
            return 0
        print(data1)
        print("---------------")
        try:
            for i in range(data1.bounding_boxes[0].num):
                if((  data1.bounding_boxes[i].xmax<2800 )and  ( data1.bounding_boxes[i].xmin>500)and data1.bounding_boxes[i].ymax>800):
                    has_people1=1
                    break
        except:
            pass
        if (not has_people1):
            _thread.start_new_thread(text_to_speech, ("I will go  point2222", ))
            self.goto(self.detect_2)
        if has_people1:
            _thread.start_new_thread(text_to_speech, ("I will fucking fucking go next point22222222", )) 

        
        has_people2=0
        try:
            data2 = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            print("no people")
            return 0
        print(data2)
        print("---------------")
        try:
            for i in range(data2.bounding_boxes[0].num):
                if((  data2.bounding_boxes[i].xmax<2800 )and  ( data2.bounding_boxes[i].xmin>500)and data2.bounding_boxes[i].ymax>800):
                    has_people2=1
                    _thread.start_new_thread(text_to_speech, ("point2 has people", ))
                    break
        except:
            pass
        if (not has_people2):
            _thread.start_new_thread(text_to_speech, ("I will go to point333", ))
            self.goto(self.detect_3)
        if has_people2:
            if has_people1:
                if(data2.bounding_boxes[i].xmax>data1.bounding_boxes[i].xmax):
                   self.goto(self.detect)
                else:
                   self.goto(self.detect1) 
        
        has_people3=0
        try:
            data3 = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            print("no people")
            return 0
        print(data3)
        print("---------------")
        try:
            for i in range(data3.bounding_boxes[0].num):
                if((  data3.bounding_boxes[i].xmax<2800 )and  ( data3.bounding_boxes[i].xmin>500)and data3.bounding_boxes[i].ymax>800):
                    has_people3=1
                    _thread.start_new_thread(text_to_speech, ("point3 has people", ))
                    break
        except:
            pass
        if has_people1:
            if (not has_people2):        
                if (not has_people3):
                    _thread.start_new_thread(text_to_speech, ("I will go to forth point", ))
                    self.goto(self.detect_4)
        if has_people1:
            if (not has_people2):        
                if has_people3:
                    if(data3.bounding_boxes[i].xmax>data1.bounding_boxes[i].xmax):
                        return 0
                    else:
                        self.goto(self.detect_1)
        if (not has_people1):
            if has_people2: 
                if (not has_people3):
                    self.goto(self.detect_4)
        if (not has_people1):
            if has_people2: 
                if has_people3:
                    if(data3.bounding_boxes[i].xmax>data2.bounding_boxes[i].xmax):
                        return 0
                    else:
                        self.goto(self.detect_2)                
        if (not has_people1):
            if (not has_people2):        
                    self.goto(self.detect_4)
        return 0

    def loc(self):

        cmd=Twist()
        cmd.linear.x=self.door_position[0]-self.postion_x
        cmd.linear.y=self.door_position[1]-self.postion_y
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1)
        cmd1=Twist()
        angular=self.door_position[2]-to_euler_angles(self.orientation_w,self.orientation_x,self.orientation_y,self.orientation_z)
        cmd1.angular.z = (angular)/180
        self.cmd_vel_pub.publish(cmd1)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
    def loc2(self):


        cmd1=Twist()
        angular=self.detect_position[2]-to_euler_angles(self.orientation_w,self.orientation_x,self.orientation_y,self.orientation_z)
        cmd1.angular.z = (angular)/180
        self.cmd_vel_pub.publish(cmd1)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())       
            
                
                
    #main part
    def main(self):
        self.pub_pan_tilt.publish(self.pan_tilt_down)

        _thread.start_new_thread(text_to_speech, ("task begin!", ))
        # 导航到门口
        self.goto(self.door_position)
        #（第一个客人）
        # 机械臂开门


        # self.loc()
        self.pub_cmd.publish("Open_door")



        data = None
        while data is None:
            try:
                data = rospy.wait_for_message("open_door/message", String, timeout=1)
            except:
                pass


        back=Twist()
        back.linear.x=-0.8  
        self.cmd_vel_pub.publish(back)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())

        text_to_speech("hi, what is your name and your favorate drink ?")

        
        try:
            guest1_answer = speech_to_text()
            self.guest1_name,self.guest1_drink= message_proc(guest1_answer)
        except:
            text_to_speech("Please say it again.")
            guest1_answer = speech_to_text()
            self.guest1_name,self.guest1_drink= message_proc(guest1_answer)

            
            
        _thread.start_new_thread(text_to_speech, ("Follow me if possible, behind my body", ))

        # 导航到客厅
        self.goto(self.detect_1)
        #self.loc2()
        self.master1_hzx()

        # 介绍主人客人 
        text_to_speech(
            "dear {} ,that is {} , and favorate drink is {}, dear {} ,This is {} , and favorate drink is {}."
            .format(self.guest1_name, self.master_name, self.master_drink,
                    self.master_name,self.guest1_name, self.guest1_drink))
        _thread.start_new_thread(text_to_speech, ("I will point to a seat you can take.", ))

        # 转向空座位
        self.empty_seat1_first()
        text_to_speech("You can sit there.")


        
        # （第二个人）
        # 导航到门口
        self.goto(self.door_position)
        # self.loc()
        # # 机械臂开门
        # self.pub_cmd.publish("Open_door")
        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message("open_door/message", String, timeout=1)
        #     except:
        #         pass
        back=Twist()
        back.linear.x=-0.8
        self.cmd_vel_pub.publish(back)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())

        text_to_speech("hi, what is your name and your favorate drink ?")
        try:
            guest2_answer = speech_to_text()
            self.guest2_name,self.guest2_drink= message_proc(guest2_answer)
        except:
            text_to_speech("Please say it again.")
            guest2_answer = speech_to_text()
            self.guest2_name,self.guest2_drink= message_proc(guest2_answer)

        _thread.start_new_thread(text_to_speech, ("follow me if possible, behind my body.", ))


        # 导航到客厅
        self.goto(self.detect_1)
        self.second()
        #介绍
        text_to_speech(
            "dear {} ,they are {} and {} , and their favorate drink are {} and{}."
            .format(self.guest2_name, self.master_name, self.guest1_name,
                    self.master_drink, self.guest1_drink))


        text_to_speech(
            "dear {} and {}  , this is {} , and favorate drink is {}.".
            format(self.guest1_name, self.master_name, self.guest2_name,
                   self.guest2_drink))

        _thread.start_new_thread(text_to_speech, ("I will point to a seat you can take.", ))
        self.goto(self.detect_1)
        self.loc2()
        # 转向空座位
        self.empty_seat1_first()
        text_to_speech("You can sit there.")
        text_to_speech("Done.")


# ----------Voice-------------------------------------------------------------------------
def text_to_speech(text):
    result = speech_synthesizer.speak_text_async(text).get()

    if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print("Speech synthesized to speaker for text [{}]".format(text))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech synthesis canceled: {}".format(
            cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            if cancellation_details.error_details:
                print("Error details: {}".format(
                    cancellation_details.error_details))
        print("Did you update the subscription info?")


def speech_to_text():
    result = speech_recognizer.recognize_once()

    # Checks result.
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print("Recognized: {}".format(result.text))
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("No speech could be recognized: {}".format(
            result.no_match_details))
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation_details = result.cancellation_details
        print("Speech Recognition canceled: {}".format(
            cancellation_details.reason))
        if cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(
                cancellation_details.error_details))
    return result.text
def message_proc(string):
    #string="My name is Hua and my favoriate drink is orange juice."

    more_string=1
    #print(string.split())
    list=string.split()

    name=list[list.index("is")+1]
    print("name is "+name)

    list2=list
    del list2[list.index("is")]

    drink = list2[list2.index("is")+1]
    for i in drink:
        if( i =="."):
            more_string=0
    if(more_string):
        drink+=" "
        drink+=list2[list2.index("is")+2]
    drink =drink.strip('.')
    print("drink is "+drink)
    return name,drink
def to_euler_angles(w, x, y, z):
    """w、x、y、z to euler angles"""
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    yaw = y*180/math.pi

    return yaw
if __name__ == "__main__":
    #Voice init
    speech_key, service_region = "b756dfeefd0f4947a88b5b0b30826943", "eastus"
    speech_config = speechsdk.SpeechConfig(subscription=speech_key,
                                           region=service_region)
    speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"
    speech_synthesizer = speechsdk.SpeechSynthesizer(
        speech_config=speech_config)
    speech_recognizer = speechsdk.SpeechRecognizer(
            speech_config=speech_config)
    #ROS
    rospy.init_node('receptionist', anonymous=True)

    receptionist_buct = receptionist()

    receptionist_buct.main()
    rospy.spin()

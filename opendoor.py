#!/usr/bin/env python3
import rospy
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
        self.pub_pan_tilt.publish(self.pan_tilt_down)

        #variable TODO
        self.master_name = 'jack'
        self.master_drink = 'orange'
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
        self.door_position = [3.506,-5.63,5]
        self.sofa_position = [-2.1,1,45]
        self.detect_position = [-2.1,1,0]

        self.postion_x =0
        self.postion_y =0
        self.orientation_x =0
        self.orientation_y =0
        self.orientation_z =0
        self.orientation_w =0
                                
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
        print (self.postion_x)
        print(self.postion_y)
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

    def empty_seat(self):
        

        linear=Twist()
        try:
            data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            return
        if(data.bounding_boxes[0].xmax<1000 and data.bounding_boxes[0].xmin>2800):
            return

        linear.linear.y=0.3
        self.cmd_vel_pub.publish(linear)
        rospy.sleep(4)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(4)


        try:
            data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            return
        if(data.bounding_boxes[0].xmax<1000 and data.bounding_boxes[0].xmin>2800):
            return

        linear.linear.y=0.3
        self.cmd_vel_pub.publish(linear)
        rospy.sleep(4)
        self.cmd_vel_pub.publish(Twist())

        try:
            data = rospy.wait_for_message("/yolov5/BoundingBoxes", BoundingBoxes, timeout=1)
        except:
            return
        if(data.bounding_boxes[0].xmax<1000 and data.bounding_boxes[0].xmin>2800):
            return
        
        linear.linear.y=-0.3
        self.cmd_vel_pub.publish(linear)
        rospy.sleep(6)
        self.cmd_vel_pub.publish(Twist())
        return
        
        # if(data.bounding_boxes[0].num==1):
        # if(data.bounding_boxes[0].num==1):
        #     print(data.bounding_boxes[0].xmin)
        #     print(data.bounding_boxes[0].xmax)
        #     print("--------------------")
        #     if(data.bounding_boxes[0].xmin>1100 and data.bounding_boxes[0].xmax<2300):
        #         angular.angular.z=-0.3
        #         self.cmd_vel_pub.publish(angular)
        #         rospy.sleep(1)
        #         self.cmd_vel_pub.publish(Twist())
        # if(data.bounding_boxes[0].num==2):
        #     for i in range(2):
        #         if(data.bounding_boxes[i].xmin<1000):
        #             seat1=0

        #         if(data.bounding_boxes[i].xmax>2800):
        #             seat3=0
        #     if(seat1):
        #         angular.angular.z=-0.3
        #         self.cmd_vel_pub.publish(angular)
        #         rospy.sleep(1)
        #         self.cmd_vel_pub.publish(Twist())
        #     if(seat3):
        #         angular.angular.z=0.3
        #         self.cmd_vel_pub.publish(angular)
        #         rospy.sleep(1)
        #         self.cmd_vel_pub.publish(Twist())
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
        print(angular)
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        
            
                
                
    #main part
    def main(self):
        # say task begin
        _thread.start_new_thread(text_to_speech, ("task begin!", ))
        while(1):
        # 导航到门口
            self.goto(self.door_position)

            #（第一个客人）
            # 机械臂开门
            self.loc()
            self.pub_cmd.publish("Open_door")
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message("test", String, timeout=1)
                except:
                    pass
            self.goto([0,0,0])





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
    speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
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
    print('dongray')
    rospy.spin()

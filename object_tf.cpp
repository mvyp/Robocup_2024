#include <cmath>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include<yolov5_ros_msgs/BoundingBoxes.h>
#include<yolov5_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


// define a class, including a constructor, member variables and member functions
class Pointcloud_filter
{
public:
    Pointcloud_filter(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired

private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber sub_img_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber sub_cmara_info_;
    ros::Subscriber bounding_box_;

    tf2_ros::TransformBroadcaster br;
    bool is_K_empty=1 ;
    bool is_IMG_empty=1 ;
    double K[9];
    int height;
    int width=2048;
    unsigned short *depth_data = new unsigned short;
    
    void img_callback(const sensor_msgs::ImageConstPtr &img_msg);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg);
    void bounding_box_callback(const  yolov5_ros_msgs::BoundingBoxes::ConstPtr &bounding_box_msg);
    
};



Pointcloud_filter:: Pointcloud_filter(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

    is_K_empty = 1;
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

  
    sub_img_ = nh_.subscribe("/depth_to_rgb/image_raw", 100, &Pointcloud_filter::img_callback,this);
    sub_cmara_info_ = nh_.subscribe("/depth_to_rgb/camera_info", 1, &Pointcloud_filter::camera_info_callback,this);
    bounding_box_ = nh_.subscribe("/yolov5/BoundingBoxes", 1, &Pointcloud_filter::bounding_box_callback,this);
}



void Pointcloud_filter::img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    // Step1: 读取深度图
    //ROS_INFO("image format: %s %dx%d", img_msg->encoding.c_str(), img_msg->height, img_msg->width);
    

    depth_data = (unsigned short*)&img_msg->data[0];


    if(is_IMG_empty){
    std::cout<<"sizeof"<<sizeof(depth_data)<<std::endl;}
    is_IMG_empty=0;
}


 
void Pointcloud_filter::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
    //ROS_INFO("Successfully read camera info!");
    // 读取相机参数
    if(is_K_empty)
    {
        height = camera_info_msg->height;
        width = camera_info_msg->width;
        for(int i=0; i<9; i++)
        {
            K[i] = camera_info_msg->K[i];
        }
        is_K_empty = 0;
         std::cout<<height<<"  "<<width<<std::endl;
    }
}


void Pointcloud_filter::bounding_box_callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr &bounding_box_msg){

if(!is_K_empty and !is_IMG_empty){
    for(int i=0;i<bounding_box_msg->bounding_boxes.size();i++){
        if(bounding_box_msg->bounding_boxes[i].probability<0.3)
        continue;

    double xmin=bounding_box_msg->bounding_boxes[i].xmin;
    double xmax=bounding_box_msg->bounding_boxes[i].xmax;
    double ymin=bounding_box_msg->bounding_boxes[i].ymin;
    double ymax=bounding_box_msg->bounding_boxes[i].ymax;
    double accx=0;
    double accy=0;
    double accz=0;
    long count=0;

    

    for(int uy=ymin+(ymax-ymin)/3; uy<ymax-(ymax-ymin)/3;  uy++){
        for(int ux=xmin+(xmax-xmin)/3; ux<xmax-(xmax-xmin)/3;  ux++){
            float z;

            z = *(depth_data + uy*width + ux) / 1000.0;   

            if(z!=0)
            {
                accx += z * (ux - K[2]) / K[0];
                accy += z * (uy - K[5]) / K[4];
                accz += z ;
                count+=1;

            }
        }  

    }
    accx/=count;
    accy/=count;
    accz/=count;
    if (isnan(accx)!=1&&isnan(accy)!=1&&isnan(accz)!=1)
    {std::cout<< bounding_box_msg->bounding_boxes[i].Class <<" "<<accx<<" "<<accy<<" "<<accz<<std::endl;


    geometry_msgs::TransformStamped transformStamped;
   
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_base";
    //transformStamped.child_frame_id = "/objects/" + bounding_box_msg->bounding_boxes[i].Class;
    transformStamped.child_frame_id =  bounding_box_msg->bounding_boxes[i].Class;

    transformStamped.transform.translation.x = accz;
    transformStamped.transform.translation.y = -accx;
    transformStamped.transform.translation.z = -accy;
 
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
 
    br.sendTransform(transformStamped); }
    }}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle n;
    ROS_INFO("Begin!1");
    Pointcloud_filter pointcloud_filter(&n);
 
    ros::spin();
    ROS_INFO("Begin!1");
    return 0;
}
#include <math.h>
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
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>


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

    ros::Publisher pub_point_cloud2_;

    bool is_K_empty=1 ;
    bool is_IMG_empty=1 ;
    double K[9];
    int height;
    int width;
    unsigned short *depth_data = new unsigned short;
    std::string deep_camera_frame;
    tf::TransformListener m_tfListener;


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
    pub_point_cloud2_ = nh_.advertise<sensor_msgs::PointCloud2>("/object_point_cloud", 1000);
}



void Pointcloud_filter::img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    // Step1: 读取深度图
    //ROS_INFO("image format: %s %dx%d", img_msg->encoding.c_str(), img_msg->height, img_msg->width);
    deep_camera_frame = img_msg->header.frame_id;


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

pcl::PointCloud<pcl::PointXYZ> pc_global;

if(!is_K_empty and !is_IMG_empty){
    for(int i=0;i<bounding_box_msg->bounding_boxes.size();i++){
        if(bounding_box_msg->bounding_boxes[i].probability<0.5)
            continue;

    double xmin=bounding_box_msg->bounding_boxes[i].xmin;
    double xmax=bounding_box_msg->bounding_boxes[i].xmax;
    double ymin=bounding_box_msg->bounding_boxes[i].ymin;
    double ymax=bounding_box_msg->bounding_boxes[i].ymax;
    double x;
    double y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    

        for(int uy=ymin; uy<ymax;  uy++){
            for(int ux=xmin; ux<xmax;  ux++){
                double z;

                z = *(depth_data + uy*width + ux) / 1000.0;   

               if(z!=0)
               {
                    x = z * (ux - K[2]) / K[0];
                    y = z * (uy - K[5]) / K[4];
                    pcl::PointXYZ p(x, y, z);
                    cloud->push_back(p);

                }
            }  
        }

    tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
      try
      {
          // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
          m_tfListener.lookupTransform("/root", deep_camera_frame, bounding_box_msg->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
      }

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
      pcl::transformPointCloud(*cloud, pc_global, sensorToWorld);   //得到世界坐标系下的点云
      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = "root"; 
      ROS_INFO("publishing");
      pub_point_cloud2_ .publish(map_cloud);  //加上时间戳和frameid发布出来


    }
}
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
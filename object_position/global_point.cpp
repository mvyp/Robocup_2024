#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pcl/common/common.h>  
#include <iostream>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <pcl/sample_consensus/model_types.h>   
#include <pcl/sample_consensus/method_types.h>   
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

#include "gpd_ros/GraspConfig.h"


class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Publisher m_globalcloudPub;  //发布局部地图点云
  ros::Publisher loca;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;  //接收点云
  tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
  tf::TransformListener m_tfListener;  // 转化坐标系

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
      
      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh,"/camera/depth/color/points", 1);    //接收点云消息
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, "/root", 100);  //接收tf和点云之后触发接收  world是frameid
      m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));   //回调函数
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/global_map", 2, true);   //发布全局地图，用于rviz展示
      loca = m_nh.advertise<gpd_ros::GraspConfig>("/cloud/goal_pose",1);
  }

  ~SegMapROSWraper()
  {
      delete m_pointCloudSub;
      delete m_tfPointCloudSub;
  }
  


  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input)  //接收到点云和tf之后，根据tf转化，然后回调函数
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ> pc_global;
    pcl::fromROSMsg(*input, *cloud);


    pcl::PassThrough<pcl::PointXYZ> px;	// 创建滤波器对象
    px.setInputCloud(cloud);				//设置输入点云
    px.setFilterFieldName("z");			//设置滤波所需字段z
    px.setFilterLimits(0, 0.4);		//设置Z字段过滤范围
    px.setFilterLimitsNegative(false);	//默认false，保留范围内的点云；true，保存范围外的点云
    px.filter(*cloud);			//执行滤波，保存滤波后点云

    // std::cout<<cloud->size()<<std::endl;
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//申明模型的参数
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//申明存储模型的内点的索引
        
    // // 创建一个分割方法
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // // 这一句可以选择最优化参数的因子
    // seg.setOptimizeCoefficients (true);
 
    // seg.setModelType (pcl::SACMODEL_PLANE);   //平面模型
    // seg.setMethodType (pcl::SAC_RANSAC);    //分割平面模型所使用的分割方法
    // //seg.setMaxIterations (100); 
    // seg.setDistanceThreshold (0.01);        //设置最小的阀值距离
  
    // seg.setInputCloud (cloud);   //设置输入的点云
    // seg.segment (*inliers,*coefficients);  
  
    // // 把提取出来的外点 -> ros发布出去
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (inliers);
    // extract.setNegative (true);
    // extract.filter (*cloud);



  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	// sor.setInputCloud(cloud_filtered);
	// sor.setRadiusSearch(0.1);
	// sor.setMinNeighborsInRadius(150);
	// sor.filter(*cloud_filter); 


    tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
    try
    {
        // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
        m_tfListener.lookupTransform("/root", input->header.frame_id, input->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
    pcl::transformPointCloud(*cloud, pc_global, sensorToWorld);   //得到世界坐标系下的点云
    //std::cout<< sensorToWorld <<std::endl;
    double x=0;
    double y=0;
    double z=0;

    std::cout<<cloud->size()<<std::endl;
    for(int i = 0; i < cloud->size(); ++i){
    x+=pc_global.at(i).x;
    y+=pc_global.at(i).y;
    z+=pc_global.at(i).z; }

    gpd_ros::GraspConfig grasp_msg;
    grasp_msg.position.x = x/(double)cloud->size();
    grasp_msg.position.y = y/(double)cloud->size();
    grasp_msg.position.z = z/(double)cloud->size();
    loca.publish(grasp_msg);


    std::cout<<x/(double)cloud->size()<<std::endl;
    std::cout<<y/(double)cloud->size()<<std::endl;
    std::cout<<z/(double)cloud->size()<<std::endl;

    
    sensor_msgs::PointCloud2 map_cloud;
    pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
    map_cloud.header.stamp = ros::Time::now();
    map_cloud.header.frame_id = "root"; 
    ROS_INFO("publishing----------------------------------------------");
    m_globalcloudPub .publish(map_cloud);  //加上时间戳和frameid发布出来
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "colored"); 
  ROS_INFO("begin");
  SegMapROSWraper  SM;

  ros::spin();
  ros::shutdown();
  return 0;
}


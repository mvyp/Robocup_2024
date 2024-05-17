// #include <ros/ros.h>
// // PCL specific includes PCL 的相关的头文件
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// //滤波的头文件
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>

// #include <pcl_ros/transforms.h>
// #include <tf/message_filter.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// //#include "gpd_ros/filter_space.h"
// class filter {
// public:
//   filter(ros::NodeHandle &n);
//   // ~filter();
// private:
//   ros::NodeHandle n_;
//   ros::Publisher pub_;
//   ros::Subscriber sub_;
//   tf::TransformListener tf_listener_;
//   tf::StampedTransform transform_;
//   void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
// };


// filter::filter(ros::NodeHandle &n) : n_(n) 
// {
//   sub_ = n_.subscribe<sensor_msgs::PointCloud2>(
//       "/camera/depth/color/points", 100, &filter::cloud_cb, this);
//   pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 100);
//       ros::Rate rate(10.0);

// }
// //回调函数
// void filter::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

//   // 声明存储原始数据与滤波后的数据的点云的格式
//   // Container for original & filtered data
//   pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 *cloud_filtered =
//       new pcl::PCLPointCloud2; //存储滤波后的数据格式
//   pcl::PCLPointCloud2Ptr filptr(cloud_filtered);

//   // 转化为PCL中的点云的数据格式
//   pcl_conversions::toPCL(*cloud_msg, *cloud);

//   // 进行一个滤波处理
//   /*pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建滤波对象
//   sor.setInputCloud (cloudPtr);  //设置输入的滤波，将需要过滤的点云给滤波对象
//   sor.setLeafSize (0.1, 0.1, 0.1);  //设置滤波时创建的体素大小为1cm立方体
//   sor.filter (*cloud_filtered);//执行滤波处理，存储输出cloud_filtered*/

//   pcl::PassThrough<pcl::PCLPointCloud2> px; // 创建滤波器对象
//   px.setInputCloud(cloudPtr);               //设置输入点云
//   px.setFilterFieldName("x");               //设置滤波所需字段z
//   px.setFilterLimits(-0.3, 0.3);            //设置Z字段过滤范围
//   // px.setFilterLimitsNegative(false);
//   // //默认false，保留范围内的点云；true，保存范围外的点云
//   px.filter(*cloud_filtered); //执行滤波，保存滤波后点云

//   px.setInputCloud(filptr);
//   px.setFilterFieldName("y");
//   px.setFilterLimits(-0.3, 0.1);
//   px.filter(*cloud_filtered);

//   px.setInputCloud(filptr);
//   px.setFilterFieldName("z");
//   px.setFilterLimits(0.3, 1.3);
//   px.filter(*cloud_filtered);

//   // 再将滤波后的点云的数据格式转换为ROS下的数据格式发布出去
//   sensor_msgs::PointCloud2 output, point_cloud2; //声明的输出的点云的格式
//   pcl_conversions::moveFromPCL(*cloud_filtered,
//                                output); //第一个参数是输入，后面的是输出
//   while (n_.ok()){
//   try {
//       tf_listener_.waitForTransform("j2s7s300_link_base", output.header.frame_id, ros::Time(0), ros::Duration(0.2) );
//       tf_listener_.lookupTransform("j2s7s300_link_base", output.header.frame_id, ros::Time(0), transform_);
//       }    
//   catch (tf::TransformException ex) {
//       ROS_ERROR("%s",ex.what());
//   }}

//   pcl_ros::transformPointCloud("j2s7s300_link_base", output, point_cloud2, tf_listener_);

//   // point_cloud2.header.frame_id = "j2s7s300_link_base";
//   point_cloud2.header.stamp = ros::Time::now();
//   pub_.publish(point_cloud2);
// }

// int main(int argc, char **argv) {
//   // Initialize ROS
//   ros::init(argc, argv, "cloud_filter"); //声明节点的名称
//   ros::NodeHandle nh;
//   filter fil(nh);
//   ros::spin();
// }
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>


class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Publisher m_globalcloudPub;  //发布局部地图点云
  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;  //接收点云
  tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
  tf::TransformListener m_tfListener;  // 转化坐标系

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
      
      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh,"/camera/depth/color/points", 100);    //接收rosbag中的点云消息
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, "/j2s7s300_link_base", 100);  //接收tf和点云之后触发接收  world是frameid
      m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));   //回调函数
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/global_map", 2, true);   //发布全局地图，用于rviz展示
  }

  ~SegMapROSWraper()
  {
      delete m_pointCloudSub;
      delete m_tfPointCloudSub;
  }
  


  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
  {
      pcl::PointCloud<pcl::PointXYZL> pc;
      pcl::PointCloud<pcl::PointXYZL> pc_global;
      pcl::fromROSMsg(*cloud, pc);

      tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
      try
      {
          // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
          m_tfListener.lookupTransform("/world", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
      }

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
      pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = "/map"; 
      m_globalcloudPub .publish(map_cloud);  //加上时间戳和frameid发布出来
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "filter"); 

  SegMapROSWraper  SM;

  ros::spin();
  return 0;
}


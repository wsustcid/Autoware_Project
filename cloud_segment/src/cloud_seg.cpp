/*
 * @Author: Shuai Wang
 * @Github: https://github.com/wsustcid
 * @Version: 0.0.0
 * @Date: 2022-04-16 22:04:49
 * @LastEditTime: 2022-04-17 19:18:30
 */

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

void cloudFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_ptr)
{ 
  /* 点云降采样 */
  // cloud_out_ptr->points.clear();
  // for (unsigned int i = 0; i<cloud_in_ptr->points.size(); i++)
  // {
  //   float distance = sqrt(pow(cloud_in_ptr->points[i].x, 2) + 
  //                         pow(cloud_in_ptr->points[i].y, 2));
  //   if (distance > 0.2)
  //   {
  //     cloud_out_ptr->points.push_back(cloud_in_ptr->points[i]);
  //   }
  // }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud_in_ptr);
  voxel.setLeafSize(0.2, 0.2, 0.2);
  voxel.filter(*cloud_out_ptr);
  
}

void cloudClip(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_ptr)
{
  /* 点云裁剪 */
  cloud_out_ptr->points.clear();
  
  for (unsigned int i=0; i<cloud_in_ptr->points.size(); i++)
  {
    if (cloud_in_ptr->points[i].z >= -1.5 && cloud_in_ptr->points[i].z <= 0.5)
    {
      if (cloud_in_ptr->points[i].y >= -10 && cloud_in_ptr->points[i].y <= 10)
      {
        cloud_out_ptr->points.push_back(cloud_in_ptr->points[i]);
      }
    }
  }

}


void cloudCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out_ptr)
{
  
}


void cloudSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out_ptr)
{
  
  // std::cout << "points: " 
  //           << cloud->width * cloud->height 
  //           << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  current_cloud_ptr = cloudIn;
  // 1. 滤波：去除临近点云+降采样
  cloudFilter(current_cloud_ptr, filtered_cloud_ptr);

  // 2. 裁剪: 上下左右
  cloudClip(filtered_cloud_ptr, clipped_cloud_ptr);

  // 3. 聚类
  cloudCluster(clipped_cloud_ptr, clustered_cloud_ptr);

  cloud_out_ptr = clipped_cloud_ptr;

}

void cloudVisualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::visualization::CloudViewer viewer("Cloud View");

  viewer.showCloud(cloud);
  
  while(!viewer.wasStopped())
  {

  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cloud_seg");

  ros::NodeHandle n;

  std::cout << argv[0] << std::endl;
  
  // 点云读取
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string file_name = "/home/ubuntu18/catkin_ws/src/autoware_project/cloud_segment/data/cloudPCD/0000000000.pcd";  
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud)== -1)
  {
    PCL_ERROR("Couldn't read file! \n");
    return -1;
  }

  // 点云处理
  pcl::PointCloud<pcl::PointXYZ>::Ptr resCloud;
  cloudSegment(cloud, resCloud);

  // 点云显示
  cloudVisualize(resCloud);

  


  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  //ros::Rate loop_rate(10);


  // int count = 0;
  // while (ros::ok())
  // {

  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();

  //   ROS_INFO("%s", msg.data.c_str());

  //   chatter_pub.publish(msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }


  return 0;
}
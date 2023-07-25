#define BOOST_BIND_NO_PLACEHOLDERS
#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include "pcl_example/pcl_example_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

double prev_error=0;
double error_sum=0;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", sensor_qos, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/curb",2);
      publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tfed",2);
      publisher_curb = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected",2);
      cmdpub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",2);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     
    }

  private:
    
     void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
    
     
      RCLCPP_INFO(this->get_logger(), "I heard");
      
      sensor_msgs::msg::PointCloud2 msg_tf;
      pcl_ros::transformPointCloud("base_link",*msg,msg_tf,*tf_buffer_);
      publisher2_->publish(msg_tf);
      
      
      //pcl::PointCloud<pcl::PointXYZ> cloudo;
      //pcl::PointCloud<pcl::PointXYZ> cloudo_out;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo_projected (new pcl::PointCloud<pcl::PointXYZ>);
      
      pcl::fromROSMsg(msg_tf, *cloudo);  
      
      std::cout << "size filterd = "<<cloudo->size(); 
      //pcl::io::savePCDFileASCII ("curb_tf_pcd.pcd", cloudo);
      
      pcl::PassThrough<pcl::PointXYZ> passo_z;
      passo_z.setInputCloud (cloudo);
      passo_z.setFilterFieldName ("z");
      passo_z.setFilterLimits (-1, 0.15);
      //pass.setNegative (true);
      passo_z.filter (*cloudo_filtered);
  
      pcl::PassThrough<pcl::PointXYZ> passo_y;
      passo_y.setInputCloud (cloudo_filtered);
      passo_y.setFilterFieldName ("y");
      passo_y.setFilterLimits (0.5, 2);
      //passy.setNegative (true);
      passo_y.filter (*cloudo_filtered);
  
      pcl::PassThrough<pcl::PointXYZ> passo_x;
      passo_x.setInputCloud (cloudo_filtered);
      passo_x.setFilterFieldName ("x");
      passo_x.setFilterLimits (0, 0.5);
      //passo_x.setNegative (true);
      passo_x.filter (*cloudo_filtered);
  
  
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (cloudo_filtered);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloudo_filtered);
   
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outr;
      outr.setInputCloud (cloudo_filtered);
      outr.setMeanK (50);
      outr.setStddevMulThresh (1.0);
      outr.filter (*cloudo_filtered);
      
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloudo_filtered);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (0.03);
      ne.compute (*cloud_normals);
  
      double nx, ny, nz;
      double nrx, nry, nrz;  
      double x, y, z;
  
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;
  
      for (auto i = 0; i < cloud_normals->size(); i++)
      {
      nx = cloud_normals->points[i].normal_x;
      ny = cloud_normals->points[i].normal_y;
      nz = cloud_normals->points[i].normal_z;
      
      double mag=sqrt(nx*nx + ny*ny + nz*nz)+0.0000000000000001;
      
      nrx=0;
      nry=-1;
      nrz=0;
      
      x = (*cloudo_filtered)[i].x;
      y = (*cloudo_filtered)[i].y;
      z = (*cloudo_filtered)[i].z;
       
      
      double theta1 = acos((ny*nry)/mag);
      theta1 = (theta1*180)/3.14;
      
      //if(theta1>80 && theta1<100)
      //if(theta1>75 && theta1<105)
      //if(theta1>70 && theta1<110)
      //if(theta1>60 && theta1<120)
      
      if(theta1>50 && theta1<130)
      {
         //remove
         // std::cout<<"entered";
         inliers->indices.push_back(i);
      }
      
      }
    
    
      extract.setInputCloud(cloudo_filtered);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*cloudo_filtered);
    
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> out2r;
      out2r.setInputCloud (cloudo_filtered);
      out2r.setMeanK (50);
      out2r.setStddevMulThresh (1.0);
      out2r.filter (*cloudo_filtered);
      
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      coefficients->values.resize (4);
      coefficients->values[0] = coefficients->values[1] = 0;
      coefficients->values[2] = 1.0;
      coefficients->values[3] = 0;
 
      // Create the filtering object
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud (cloudo_filtered);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloudo_projected);

      float offset=0.75;
      double centroidx=0,centroidy=0,centroidz=0;
  

      for (auto& point: *cloudo_projected)
      {
    
       point.y = point.y-offset;
       centroidx+=point.x;
       centroidy+=point.y;
       centroidz+=point.z;
    
      }
      centroidx=centroidx/cloudo_filtered->size();
      centroidy=centroidy/cloudo_filtered->size();
      centroidz=centroidz/cloudo_filtered->size();
  
      //std::cout << "centroidx = "<<centroidx<<'\n';
      std::cout << "centroidy = "<<centroidy<<'\n';
      //std::cout << "centroidz = "<<centroidz<<'\n';
      //std::cout << "size filterd = "<<cloudo_filtered->size(); 

            
      sensor_msgs::msg::PointCloud2 msg_out;
      pcl::toROSMsg(*cloudo_filtered, msg_out);
      msg_out.header.frame_id = msg_tf.header.frame_id;
      msg_out.header.stamp = msg_tf.header.stamp;
      publisher_->publish(msg_out);
      
      sensor_msgs::msg::PointCloud2 curb_out;
      pcl::toROSMsg(*cloudo_projected, curb_out);
      msg_out.header.frame_id = msg_tf.header.frame_id;
      msg_out.header.stamp = msg_tf.header.stamp;
      publisher_curb->publish(curb_out);
    
      double ref = 0;
      double error= ref-centroidy;
      
      double kp=1,kd=0.001,ki=0;
      double ang_vel_z;
      double vel_forward=0.5;
      
      error_sum=error_sum+error;
      
      ang_vel_z= kp*error+kd*(error-prev_error)+ki*(error_sum);
      
      prev_error=error;
      
      std::cout<<"velocity_z "<<ang_vel_z<<'\n';
      
      geometry_msgs::msg::Twist cmd_vel_msg;
      
      cmd_vel_msg.linear.x = vel_forward;
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_msg.linear.z = 0.0;

      cmd_vel_msg.angular.x = 0.0;
      cmd_vel_msg.angular.y = 0.0;
      cmd_vel_msg.angular.z = -ang_vel_z;
      
      cmdpub_->publish(cmd_vel_msg);
      
      
      
      
  
      
     
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_curb;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdpub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

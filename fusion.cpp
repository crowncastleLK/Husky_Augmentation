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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl_ros/transforms.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"


#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::placeholders::_1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo (new pcl::PointCloud<pcl::PointXYZ>);
class Fusion_node : public rclcpp::Node
{
  public:
    Fusion_node()
    : Node("Lidar_camera_fusion_node")
    {
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", sensor_qos, std::bind(&Fusion_node::pc_callback, this, _1));
      
      // /camera/image_raw
      
     image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",10, std::bind(&Fusion_node::image_callback, this, _1));
      
      tfed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_camera_frame",2);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     
    }

  private:
    
     void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
    
      RCLCPP_INFO(this->get_logger(), "I heard pointcloud");
      
      sensor_msgs::msg::PointCloud2 msg_tf;
      pcl_ros::transformPointCloud("os_lidar",*msg,msg_tf,*tf_buffer_);
      tfed_publisher_->publish(msg_tf);
      
    
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudo (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(msg_tf, *cloudo);  
      
      
      //pcl::io::savePCDFileASCII ("curb_tf_pcd.pcd", cloudo);


      
    }
    
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
            
         /*extrinsics: ros2 run tf2_ros tf2_echo os_lidar camera_color_optical_frame
         At time 0.0
         - Translation: [-0.121, -0.033, -0.127]
         - Rotation: in Quaternion [-0.270, -0.271, 0.654, 0.652]

         */
         
         /*intrinsics 
         run rs-enumerate-devices -c in terminal
          Intrinsic of "Color" / 1280x720 / {YUYV/RGB8/BGR8/RGBA8/BGRA8/Y16}
          Width:      	1280
          Height:     	720
          PPX:        	634.006408691406
          PPY:        	359.488098144531
          Fx:         	907.704406738281
          Fy:         	907.281127929688
          Distortion: 	Inverse Brown Conrady
          Coeffs:     	0  	0  	0  	0  	0  
          FOV (deg):  	70.37 x 43.29
        */

         cv::Mat R(3, 3, CV_64F);
         cv::Mat T(3, 1, CV_64F);
                  
         double extrinsics_rx=0.0033801;
         double extrinsics_ry=-0.7854413;
         double extrinsics_rz=1.5752593;
         double extrinsics_tx=-0.121;
         double extrinsics_ty=-0.033;
         double extrinsics_tz=-0.127;
         
         double cosRx = cos(extrinsics_rx);
         double sinRx = sin(extrinsics_rx);
         double cosRy = cos(extrinsics_ry);
         double sinRy = sin(extrinsics_ry);
         double cosRz = cos(extrinsics_rz);
         double sinRz = sin(extrinsics_rz);

         R.at<double>(0, 0) = cosRz * cosRy;
         R.at<double>(0, 1) = cosRz * sinRy * sinRx - sinRz * cosRx;
         R.at<double>(0, 2) = cosRz * sinRy * cosRx + sinRz * sinRx;
         R.at<double>(1, 0) = sinRz * cosRy;
         R.at<double>(1, 1) = sinRz * sinRy * sinRx + cosRz * cosRx;
         R.at<double>(1, 2) = sinRz * sinRy * cosRx - cosRz * sinRx;
         R.at<double>(2, 0) = -sinRy;
         R.at<double>(2, 1) = cosRy * sinRx;
         R.at<double>(2, 2) = cosRy * cosRx;

         T.at<double>(0) = extrinsics_tx;
         T.at<double>(1) = extrinsics_ty;
         T.at<double>(2) = extrinsics_tz;
        
         double intrinsics_fx=907.704406738281;
         double intrinsics_fy=907.281127929688;
         double intrinsics_cx=634.006408691406;
         double intrinsics_cy=359.488098144531;
         
      
        cv::Mat rgbImage=cv_bridge::toCvShare(msg, "bgr8")->image;
        // Project LiDAR points to camera image
         for (auto& point: *cloudo)
      {
      
        //RCLCPP_INFO(this->get_logger(), "In transform loop");
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat point3DTransformed = R * point3D + T;

        double x = point3DTransformed.at<double>(0);
        double y = point3DTransformed.at<double>(1);
        double z = point3DTransformed.at<double>(2);

        // Skip points that are behind the camera
        if (z <= 0) {
            continue;
        }

        int u = static_cast<int>(intrinsics_fx * x / z + intrinsics_cx);
        int v = static_cast<int>(intrinsics_fy * y / z + intrinsics_cy);

        // Check if the projected point is within the image boundaries
        if (u >= 0 && u < rgbImage.cols && v >= 0 && v < rgbImage.rows) {
        
            double depth = sqrt(x * x + y * y + z * z);
            double maxDepth=3;
            double minDepth=0;
            // Normalize depth to a color range (e.g., blue for near, red for far)
            double normalizedDepth = (depth - minDepth) / (maxDepth - minDepth); // Adjust minDepth and maxDepth as needed
            int blue = static_cast<int>(255 * (1 - normalizedDepth));
            int red = static_cast<int>(255 * normalizedDepth);

            rgbImage.at<cv::Vec3b>(v, u) = cv::Vec3b(blue, 0, red);
        }
        
        
     }
     
     cv::imshow("view", rgbImage);
     cv::waitKey(10);
     
     
     
  
  
    }
   
   
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tfed_publisher_;
  
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fusion_node>());
  rclcpp::shutdown();
  return 0;
}

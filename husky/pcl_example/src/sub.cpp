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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl_ros/transforms.hpp>
 #include <pcl/filters/passthrough.h>

using std::placeholders::_1;


int   x_range_up = 30;
int x_range_down = 0;
int  y_range_up = 20;
int y_range_down = -10;
float z_range_down = -1.5;

int ring_idx[128];


struct EIGEN_ALIGN16 OusterPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
bool comp_up(const OusterPoint &a, const OusterPoint &b) {
  return a.y < b.y;
}

// Descending. (For points where y is less than 0)
bool comp_down(const OusterPoint &a, const OusterPoint &b) {
  return a.y > b.y;
}

     // memcpy(ring_idx, ring_idx_, sizeof(ring_idx_));
class curbDetector
{
  public:
    curbDetector(){}

    pcl::PointCloud<OusterPoint> detector(const std::vector<pcl::PointCloud<OusterPoint>> &input) {
      pc_in = input;

      // Process and detect each layer. Since we took 10 layers of 64 lidar here, the number of layers here is 10.
      for (int i = 0; i < 128; ++i) {
        pcl::PointCloud<OusterPoint> pointsInTheRing = pc_in[i]; // Save the points on this line.
        pcl::PointCloud<OusterPoint> pc_left; // Store the point where y is greater than 0 (the left point).
        pcl::PointCloud<OusterPoint> pc_right; // Store the point where y is less than 0 (the right point).

        pcl::PointCloud<OusterPoint> cleaned_pc_left;
        pcl::PointCloud<OusterPoint> cleaned_pc_right;

        OusterPoint point;
        size_t numOfPointsInTheRing = pointsInTheRing.size();

        // Separate the left and right points and save them to the corresponding point cloud.
        for (int idx = 0; idx < numOfPointsInTheRing; idx++) {
          point = pointsInTheRing[idx];
          if (point.y >= 0) {
            pc_left.push_back(point);
          } else {
            pc_right.push_back(point);
          }
        }

        // Sort. (In ascending order of absolute value)
        sort(pc_left.begin(), pc_left.end(), comp_up);
        sort(pc_right.begin(), pc_right.end(), comp_down);

        slideForGettingPoints(pc_left, true);
        slideForGettingPoints(pc_right, false);
      }

      //return curb_left + curb_right;
      return curb_right;
    }

    void slideForGettingPoints(pcl::PointCloud<OusterPoint> points, bool isLeftLine) {
      int w_0 = 10;
      int w_d = 30;
      int i = 0;

      // some important parameters influence the final performance.
      float xy_thresh = 0.3;
      float z_thresh = 0.08;

      int points_num = points.size();

      while((i + w_d) < points_num) {
        float z_max = points[i].z;
        float z_min = points[i].z;

        int idx = 0;
        float z_dis = 0;

        for (int j = 0; j < w_d; j++) {
          float dis = fabs(points[i+j].z - points[i+j+1].z);
          if (dis > z_dis) {
            z_dis = dis; idx = i+j;
          }
          if (points[i+j].z < z_min) {
            z_min = points[i+j].z;
          }
          if (points[i+j].z > z_max) {
            z_max = points[i+j].z;
          }
        }

        if (fabs(z_max - z_min) >= z_thresh) {
          for (int j = 0; j < (w_d - 1); j++) {
            float p_dist = sqrt(((points[i+j].y - points[i+1+j].y) * (points[i+j].y - points[i+1+j].y))
                              + ((points[i+j].x - points[i+1+j].x) * (points[i+j].x - points[i+1+j].x)));
            if (p_dist >= xy_thresh) {
              if (isLeftLine) {
                curb_left.push_back(points[i+j]);
                return;
              } else {
                curb_right.push_back(points[i+j]);
                return;
              }
            }
          }
          if (isLeftLine) {
            curb_left.push_back(points[idx]);
            return;
          } else {
            curb_right.push_back(points[idx]);
            return;
          }
        }
        i += w_0;
      }
    }

  private:
    std::vector<pcl::PointCloud<OusterPoint>> pc_in;
    pcl::PointCloud<OusterPoint> curb_left;
    pcl::PointCloud<OusterPoint> curb_right;
};

std::vector<pcl::PointCloud<OusterPoint>> cleanPoints(pcl::PointCloud<OusterPoint> &pc) {
      //  this function cleans the points, this way you can have only the 10 layers we want
      // 1. gets rid of unwanted layers
      // 2. takes the points according to the layer
      // 3. save the new layers in a new pointcloud mesage and return them
      size_t cloudSize = pc.size();
      // size_t ringSize;
      OusterPoint point;
      int scanID_;
      // pcl::PointCloud<OusterPoint> _laserCloud;
      std::vector<pcl::PointCloud<OusterPoint>> laserCloudScans(128);

      for (int i = 0; i < cloudSize; i++) {
        point.x = pc[i].x;
        point.y = pc[i].y;
        point.z = pc[i].z;


        scanID_ = pc[i].ring;

        for (int ring_num = 0;ring_num < 128; ring_num++) {
          if (scanID_ == ring_idx[ring_num]) {
            laserCloudScans[ring_num].push_back(point);
          }
        }
      }

      return laserCloudScans;
    }

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
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     
    }

  private:
    
     void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
    
      curbDetector cd;
      RCLCPP_INFO(this->get_logger(), "I heard");
      
      sensor_msgs::msg::PointCloud2 msg_tf;
      pcl_ros::transformPointCloud("base_link",*msg,msg_tf,*tf_buffer_);
      //publisher2_->publish(msg_tf);
      
      
      pcl::PointCloud<OusterPoint> cloudo;
      pcl::PointCloud<OusterPoint> cloudo_out;
      pcl::fromROSMsg(msg_tf, cloudo);  
      pcl::io::savePCDFileASCII ("curb_tf_pcd.pcd", cloudo);
      std::vector<pcl::PointCloud<OusterPoint>> clean_vec;
      clean_vec=cleanPoints(cloudo);
      cloudo_out = cd.detector(clean_vec);
      
      sensor_msgs::msg::PointCloud2 msg_out;

      pcl::toROSMsg(cloudo_out, msg_out);
      
      msg_out.header.frame_id = msg_tf.header.frame_id;
      msg_out.header.stamp = msg_tf.header.stamp;
      publisher_->publish(msg_out);
      
      //pcl::PassThrough<OusterPoint> passo_z;
      pcl::PassThrough<pcl::PointXYZ> pass;
      
      /*
      RCLCPP_INFO(this->get_logger(), "I heard");
      
      sensor_msgs::msg::PointCloud2 msg_tf;
      pcl_ros::transformPointCloud("base_link",*msg,msg_tf,*tf_buffer_);
      publisher2_->publish(msg_tf);
      
      pcl::PointCloud<OusterPoint>::Ptr cloudo (new pcl::PointCloud<OusterPoint>);
      pcl::PointCloud<OusterPoint>::Ptr cloudo_out (new pcl::PointCloud<OusterPoint>);
      pcl::PointCloud<OusterPoint>::Ptr cloudo_filtered (new pcl::PointCloud<OusterPoint>);
      pcl::fromROSMsg(msg_tf, *cloudo);  
      
        pcl::PassThrough<OusterPoint> passo_z;
     pcl::PassThrough<OusterPoint> passo_z;
     passo_z.setInputCloud (cloudo);
     passo_z.setFilterFieldName ("z");
     passo_z.setFilterLimits (-1, 1.0);
     //pass.setNegative (true);
     passo_z.filter (*cloudo_filtered);
     */
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher2_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char * argv[])
{

    int k;
  for(k=0;k<128;k++)
  {ring_idx[k]=k;}

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

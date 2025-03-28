#include <Eigen/Core>
#include <memory>
#include <utility>
#include <vector>

// Patchwork++-ROS
#include "GroundSegmentationServer.hpp"
#include "Utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

// mios 
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
namespace patchworkpp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

GroundSegmentationServer::GroundSegmentationServer(const rclcpp::NodeOptions &options)
  : rclcpp::Node("patchworkpp_node", options) {

  patchwork::Params params;
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  params.num_iter      = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr       = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts   = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds      = declare_parameter<double>("th_seeds", params.th_seeds);

  params.th_dist    = declare_parameter<double>("th_dist", params.th_dist);
  params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
  params.th_dist_v  = declare_parameter<double>("th_dist_v", params.th_dist_v);

  params.max_range       = declare_parameter<double>("max_range", params.max_range);
  params.min_range       = declare_parameter<double>("min_range", params.min_range);
  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.verbose = get_parameter<bool>("verbose", params.verbose);

  // ToDo. Support intensity
  params.enable_RNR = false;

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_topic", rclcpp::SensorDataQoS(),
    std::bind(&GroundSegmentationServer::EstimateGround, this, std::placeholders::_1));

  /*
   * We use the following QoS setting for reliable ground segmentation.
   * If you want to run Patchwork++ in real-time and real-world operation,
   * please change the QoS setting
   */
//  rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  cloud_publisher_     = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/cloud", qos);
  ground_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/ground", qos);
  nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/patchworkpp/nonground", qos);

  RCLCPP_INFO(this->get_logger(), "Patchwork++ ROS 2 node initialized");
}

void GroundSegmentationServer::EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const auto &cloud = patchworkpp_ros::utils::PointCloud2ToEigenMat(msg);

  // Estimate ground
  Patchworkpp_->estimateGround(cloud);
  cloud_publisher_->publish(patchworkpp_ros::utils::EigenMatToPointCloud2(cloud, msg->header));
  // Get ground and nonground
  /*
  Eigen::MatrixX3f ground     = Patchworkpp_->getGround();
  Eigen::MatrixX3f nonground  = Patchworkpp_->getNonground();
  double           time_taken = Patchworkpp_->getTimeTaken();
  PublishClouds(ground, nonground, msg->header);
  */

  //Para experimentos, junta los indices de ground y no ground
  Eigen::VectorXi ground_indx     = Patchworkpp_->getGroundIndices();
  Eigen::VectorXi non_ground_indx  = Patchworkpp_->getNongroundIndices();

  // Create a new VectorXi with combined size
  /*
  Eigen::VectorXi merged_indx(ground_indx.size() + non_ground_indx.size());
  // Copy values into the new vector
  merged_indx << ground_indx, non_ground_indx;
  PublishFullClouds(ground_indx, merged_indx, msg);
  */
  PublishFullClouds(ground_indx, non_ground_indx, msg);

}
void GroundSegmentationServer::PublishFullClouds(
  const Eigen::VectorXi ground_indices,
  const Eigen::VectorXi nonground_indices,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

  //generate non_ground_output by memcpy the original messages with the non_ground index
  sensor_msgs::msg::PointCloud2 non_ground_output;
  non_ground_output.header = msg->header;
  non_ground_output.height = 1;
  non_ground_output.width = nonground_indices.size();
  non_ground_output.is_bigendian = msg->is_bigendian;
  non_ground_output.point_step = msg->point_step;
  non_ground_output.row_step = non_ground_output.point_step * non_ground_output.width;
  non_ground_output.fields = msg->fields;
  non_ground_output.is_dense = msg->is_dense;
  non_ground_output.data.resize(non_ground_output.row_step);
  
  for (size_t i = 0; i < nonground_indices.size(); ++i) {
      int idx = nonground_indices[i];
      if (idx < 0 || idx >= static_cast<int>(msg->width * msg->height)) std::cout << "Ground Indices:" << std::endl;
      std::memcpy(&non_ground_output.data[i * non_ground_output.point_step], &msg->data[idx * msg->point_step], msg->point_step);
  }


  //generate ground_output by memcpy the original messages with the non_ground index
  sensor_msgs::msg::PointCloud2 ground_output;
  ground_output.header = msg->header;
  ground_output.height = 1;
  ground_output.width = nonground_indices.size();
  ground_output.is_bigendian = msg->is_bigendian;
  ground_output.point_step = msg->point_step;
  ground_output.row_step = ground_output.point_step * ground_output.width;
  ground_output.fields = msg->fields;
  ground_output.is_dense = msg->is_dense;
  ground_output.data.resize(ground_output.row_step);
  
  for (size_t i = 0; i < ground_indices.size(); ++i) {
      int idx = ground_indices[i];
      if (idx < 0 || idx >= static_cast<int>(msg->width * msg->height)) std::cout << "Ground Indices:" << std::endl;
      std::memcpy(&ground_output.data[i * ground_output.point_step], &msg->data[idx * msg->point_step], msg->point_step);
  }


  // Publish the filtered point clouds
  nonground_publisher_->publish(non_ground_output);
  ground_publisher_->publish(ground_output);
}



/*
void GroundSegmentationServer::PublishClouds(const Eigen::MatrixX3f &est_ground,
                                             const Eigen::MatrixX3f &est_nonground,
                                             const std_msgs::msg::Header header_msg) {

  std_msgs::msg::Header header = header_msg;
  header.frame_id = base_frame_;
  ground_publisher_->publish(std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(est_ground, header)));
  nonground_publisher_->publish(std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(est_nonground, header)));
}
  */
}  // namespace patchworkpp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(patchworkpp_ros::GroundSegmentationServer)

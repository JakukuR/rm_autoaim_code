// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/rune.hpp"

namespace rm_auto_aim
{

enum AimMode {NORMAL = 0, SMALL_BUFF = 2, LARGE_BUFF = 3};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void savevideoCallback(const sensor_msgs::msg::Image::ConstSharedPtr record_msg);

  // Armor Detector
  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  // Rune Detector
  std::unique_ptr<RuneDetector> initRuneDetector();
  struct Rune detectRune(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, const int mode);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  // Rune Detector
  std::unique_ptr<RuneDetector> rune_detector_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Detected rune publisher
  auto_aim_interfaces::msg::Rune rune_msg_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Aim mode
  int aim_mode_ = AimMode::NORMAL;
  std::shared_ptr<rclcpp::ParameterEventHandler> mode_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> mode_cb_handle_;

  // Rune part
  rclcpp::Publisher<auto_aim_interfaces::msg::Rune>::SharedPtr rune_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr save_video_sub_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_ = false;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  image_transport::Publisher rune_debug_pub_;
  // image_transport::Publisher rune_fan_pub_;

};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_

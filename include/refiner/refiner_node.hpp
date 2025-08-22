#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

#include "utils/utils.hpp"
#include "intelligent_robot_vision/msg/bounding_box.hpp"
#include "intelligent_robot_vision/msg/pan_tilt.hpp"
#include <intelligent_humanoid_interfaces/msg/vision2_master_msg.hpp>

// #include <visualization_msgs/msg/marker.hpp>

struct DetectionResult
{
  int class_id;
  float score;
  cv::Rect bbox;
};

class RefinerNode : public rclcpp::Node
{
public:
  RefinerNode();

private:
  cv::Mat K_M, D_M, R_M, P_M;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;

  int tilt_deg = 90;

  double line_delta = -999;

  cv::Mat bgr_image;

  cv::Mat latest_depth_;
  std::mutex depth_mutex_;

  std::vector<DetectionResult> Detections_ball_;
  std::vector<DetectionResult> Detections_goal_;
  std::vector<DetectionResult> Detections_hurdle_;
  std::vector<DetectionResult> Detections_line_;

  std::vector<cv::Point2f> Vpt;
  cv::Point2f pt1;
  cv::Point2f pt2;

  // ===== ROS 통신 =====
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<intelligent_robot_vision::msg::BoundingBox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<intelligent_robot_vision::msg::PanTilt>::SharedPtr pan_tilt_sub_;

  intelligent_humanoid_interfaces::msg::Vision2MasterMsg vision;
  rclcpp::Publisher<intelligent_humanoid_interfaces::msg::Vision2MasterMsg>::SharedPtr vision_pub_;

  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // ===== Callback =====
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void bboxCallback(const intelligent_robot_vision::msg::BoundingBox::SharedPtr msg);
  void pan_tilt_Callback(const intelligent_robot_vision::msg::PanTilt::SharedPtr msg);

  // ===== Data Processing =====
  void bboxProcessing();
  float dist2D(int u, int v);

  // ===== 좌표 =====
  cv::Point3f pixelToCamCoords(int u, int v);
  float groundDistance(const cv::Point3f &cam_pt);

  // void publishBallMarkerWorldMm(const cv::Point3f &world_mm, const std::string &world_frame);
};

#include <rclcpp/rclcpp.hpp>

//#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "intelligent_robot_vision/msg/pan_tilt.hpp"
#include <intelligent_humanoid_interfaces/msg/master2_vision_msg.hpp>

class PanTiltNode : public rclcpp::Node
{
public:
  PanTiltNode(); // 생성자

private:
  int pan_Pos_ = 0, tilt_Pos_ = 0;
  int mode = 2;

  // ===== ROS 통신 =====
  rclcpp::Subscription<intelligent_humanoid_interfaces::msg::Master2VisionMsg>::SharedPtr pan_tilt_sub_;

  //rclcpp::Publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>::SharedPtr Motor_Pub;
  intelligent_robot_vision::msg::PanTilt pan_tilt;
  rclcpp::Publisher<intelligent_robot_vision::msg::PanTilt>::SharedPtr pan_tilt_pub_;

  // ===== Callback =====
  void pantiltCallback(const intelligent_humanoid_interfaces::msg::Master2VisionMsg::SharedPtr msg);

  // ===== Data Processing =====
  void pan_tilt_mode();
  void pan_tilt_publish();
};

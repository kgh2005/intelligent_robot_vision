#include "pan_tilt/pan_tilt_node.hpp"

PanTiltNode::PanTiltNode() : rclcpp::Node("pan_tilt_node")
{
  // Motor_Pub = this->create_publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>("pan_tilt_dxl", 10);
  pan_tilt_pub_ = this->create_publisher<intelligent_robot_vision::msg::PanTilt>("/PanTilt", 10);

  RCLCPP_INFO(this->get_logger(), "pan_tilt_node started.");
}

void PanTiltNode::pan_tilt_publish()
{
  intelligent_robot_vision::msg::PanTilt pan_tilt;

  pan_tilt.pan = pan_Pos_;
  pan_tilt.tilt = tilt_Pos_;
  pan_tilt_pub_->publish(pan_tilt);

  // RCLCPP_INFO(this->get_logger(), "========== Pan_Tilt ==========");
  // RCLCPP_INFO(this->get_logger(), "Pan: %d, Tilt: %d", pan_Pos_, tilt_Pos_);
  // RCLCPP_INFO(this->get_logger(), " ");
}

void PanTiltNode::pan_tilt_mode()
{
  switch (mode)
  {
  case 0: // init
  {
    pan_Pos_ = 0;
    tilt_Pos_ = 45;
    pan_tilt_publish();
    break;
  }

  case 1: // 공 - 트레킹모드 - tilt 45도
  {
    pan_Pos_ = 0;
    tilt_Pos_ = 45;
    pan_tilt_publish();
    break;
  }

  case 2: // 골대 - 트레킹모드 - tilt 0도
  {
    pan_Pos_ = 0;
    tilt_Pos_ = 0;
    pan_tilt_publish();
    break;
  }

  case 3: // 허들 - 트레킹모드 - tilt 90도
  {
    pan_Pos_ = 0;
    tilt_Pos_ = -90;
    pan_tilt_publish();
    break;
  }

  default:
    RCLCPP_ERROR(this->get_logger(), "===== Pan_Tilt ERROR!! =====");
    break;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltNode>());
  rclcpp::shutdown();
  return 0;
}

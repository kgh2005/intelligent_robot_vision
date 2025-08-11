#include "refiner/refiner_node.hpp"

RefinerNode::RefinerNode() : Node("refiner_node")
{
  bbox_sub_ = this->create_subscription<intelligent_robot_vision::msg::BoundingBox>(
      "/Bounding_box",
      10,
      std::bind(&RefinerNode::bboxCallback, this, std::placeholders::_1));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/realsense/color/image_raw", 10,
      std::bind(&RefinerNode::imageCallback, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/realsense/aligned_depth_to_color/image_raw", 10,
      std::bind(&RefinerNode::depthCallback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/realsense/color/camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
      {
        K_M = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
        D_M = cv::Mat(1, msg->d.size(), CV_64F, (void *)msg->d.data()).clone();
        R_M = cv::Mat(3, 3, CV_64F, (void *)msg->r.data()).clone();
        P_M = cv::Mat(3, 4, CV_64F, (void *)msg->p.data()).clone();

        fx = K_M.at<double>(0, 0);
        fy = K_M.at<double>(1, 1);
        cx = K_M.at<double>(0, 2);
        cy = K_M.at<double>(1, 2);
      });

  RCLCPP_INFO(this->get_logger(), "RefinerNode initialized.");
}

// cv::Point3f RefinerNode::image2CameraCoords(int u, int v, float depth_m)
// {
//   if (depth_m <= 0.0f)
//     return cv::Point3f(NAN, NAN, NAN);

//   cv::Mat K_inv = K_M.inv();
//   cv::Mat uv1 = (cv::Mat_<double>(3, 1) << u, v, 1.0);
//   cv::Mat cam_coords = K_inv * uv1 * depth_m;

//   return cv::Point3f(
//       static_cast<float>(cam_coords.at<double>(0)),
//       static_cast<float>(cam_coords.at<double>(1)),
//       static_cast<float>(cam_coords.at<double>(2)));
// }

// cv::Point3f DetectionNode::computeObjectPosition(const cv::Rect &bbox)
// {
//   cv::Point3f sum(0, 0, 0);
//   int valid_count = 0;

//   std::lock_guard<std::mutex> lock(depth_mutex_);

//   if (latest_depth_.empty())
//     return cv::Point3f(NAN, NAN, NAN);

//   // 중심 찾기
//   for (int y = bbox.y; y < bbox.y + bbox.height; ++y)
//   {
//     for (int x = bbox.x; x < bbox.x + bbox.width; ++x)
//     {
//       if (x < 0 || y < 0 || x >= latest_depth_.cols || y >= latest_depth_.rows)
//         continue;

//       uint16_t depth_mm = latest_depth_.at<uint16_t>(y, x);
//       float depth_m = depth_mm * 0.001f;

//       if (depth_m <= 0.0f)
//         continue;

//       cv::Point3f pt3d = image2CameraCoords(x, y, depth_m);

//       if (!std::isnan(pt3d.x))
//       {
//         sum += pt3d;
//         valid_count++;
//       }
//     }
//   }

//   if (valid_count == 0)
//     return cv::Point3f(NAN, NAN, NAN);

//   return sum * (1.0f / valid_count);
// }

cv::Point3f RefinerNode::pixelToCamCoords(int u, int v)
{
  // float depth = latest_depth_.at<uint16_t>(v, u) * 0.001f; // m
  float depth = latest_depth_.at<uint16_t>(v, u); // mm

  // 1. 픽셀 -> 카메라 좌표계
  float x = (u - cx) * depth / fx;
  float y = (v - cy) * depth / fy;
  float z = depth;
  return {x, y, z};
}

float RefinerNode::groundDistance(const cv::Point3f &cam_pt)
{
  // 2. 카메라 tilt 보정 (X축 기준 회전)
  float tilt_rad = tilt_deg * M_PI / 180.0f; // deg → rad
  cv::Matx33f R = {
      1, 0, 0,
      0, cos(tilt_rad), -sin(tilt_rad),
      0, sin(tilt_rad), cos(tilt_rad)};
  cv::Point3f world_pt = R * cam_pt;

  // 3. 바닥면 2D 거리 (X-Z 평면)
  float distance_2d = std::sqrt(world_pt.x * world_pt.x + world_pt.z * world_pt.z);

  return distance_2d;
}

float RefinerNode::dist2D(int u, int v)
{
  cv::Point3f cam_pt = pixelToCamCoords(u, v);
  return groundDistance(cam_pt);
}

void RefinerNode::bboxProcessing()
{
  if (bgr_image.empty())
    return;

  if (Detections_ball_.size() > 0)
  {
    const auto &bbox = Detections_ball_[0].bbox; // 첫 번째 객체의 bbox

    int u = bbox.x + bbox.width / 2;
    int v = bbox.y + bbox.height / 2;

    cv::Point3f cam_pt = pixelToCamCoords(u, v); // 카메라 좌표계로 변환
    float distance_2d = dist2D(u, v);            // 2D 거리

    // RCLCPP_INFO(this->get_logger(), "========== Ball ==========");
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", cam_pt.x, cam_pt.y, cam_pt.z);
    // RCLCPP_INFO(this->get_logger(), "2D distance: %f", distance_2d);
  }
  if (Detections_goal_.size() > 0)
  {
    const auto &bbox = Detections_goal_[0].bbox; // 첫 번째 객체의 bbox

    int u = bbox.x + bbox.width / 2;
    int v = bbox.y + bbox.height / 2;

    cv::Point3f cam_pt = pixelToCamCoords(u, v); // 카메라 좌표계로 변환
    float distance_2d = dist2D(u, v);            // 2D 거리

    // RCLCPP_INFO(this->get_logger(), "========== goal ==========");
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", cam_pt.x, cam_pt.y, cam_pt.z);
    // RCLCPP_INFO(this->get_logger(), "2D distance: %f", distance_2d);
  }
  if (Detections_hurdle_.size() > 0)
  {
    const auto &bbox = Detections_hurdle_[0].bbox; // 첫 번째 객체의 bbox

    int u = bbox.x + bbox.width / 2;
    int v = bbox.y + bbox.height / 2;

    cv::Point3f cam_pt = pixelToCamCoords(u, v); // 카메라 좌표계로 변환
    float distance_2d = dist2D(u, v);            // 2D 거리

    // RCLCPP_INFO(this->get_logger(), "========== Hurdle ==========");
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", cam_pt.x, cam_pt.y, cam_pt.z);
    // RCLCPP_INFO(this->get_logger(), "2D distance: %f", distance_2d);
  }
  if (Detections_line_.size() > 0)
  {
    for (const auto &d : Detections_line_)
    {
      cv::Point2f p;
      const cv::Rect &r = d.bbox;

      p.x = r.x;
      p.y = r.y;
      Vpt.push_back(p);
    }

    cv::Vec4f bestLine = ransacLine(Vpt);
    pt1.x = bestLine[2] - bestLine[0] * 100;
    pt1.y = bestLine[3] - bestLine[1] * 100;
    pt2.x = bestLine[2] + bestLine[0] * 100;
    pt2.y = bestLine[3] + bestLine[1] * 100;

    line_delta = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
    line_delta *= (180 / M_PI);

    cv::line(bgr_image, pt1, pt2, cv::Scalar(255, 0, 0), 10);

    // RCLCPP_INFO(this->get_logger(), "========== Line ==========");
    // RCLCPP_INFO(this->get_logger(), "Line_delta: %f", line_delta);
    // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", pt2.x, pt2.y);
  }

  cv::imshow("Line", bgr_image);
  cv::waitKey(1);
}

void RefinerNode::pan_tilt_Callback(const intelligent_robot_vision::msg::PanTilt::SharedPtr msg)
{
  tilt_deg = msg->tilt;
}

void RefinerNode::bboxCallback(const intelligent_robot_vision::msg::BoundingBox::SharedPtr msg)
{
  Detections_ball_.clear();
  Detections_goal_.clear();
  Detections_line_.clear();
  Detections_hurdle_.clear();

  size_t num_boxes = msg->class_ids.size();

  for (size_t i = 0; i < num_boxes; i++)
  {
    DetectionResult det;
    det.class_id = msg->class_ids[i];
    det.score = msg->score[i];
    det.bbox = cv::Rect(msg->x1[i], msg->y1[i], msg->x2[i] - msg->x1[i], msg->y2[i] - msg->y1[i]);

    if (det.class_id == 0)
    {
      Detections_ball_.push_back(det);
    }
    else if (det.class_id == 1)
    {
      Detections_goal_.push_back(det);
    }
    else if (det.class_id == 2)
    {
      Detections_hurdle_.push_back(det);
    }
    else if (det.class_id == 3)
    {
      Detections_line_.push_back(det);
    }
  }

  if (num_boxes != 0)
  {
    bboxProcessing();
  }
}

void RefinerNode::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try
  {
    // depth 이미지를 cv::Mat으로 변환
    auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat depth_image = cv_ptr->image;

    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = depth_image.clone();
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void RefinerNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try
  {
    bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefinerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

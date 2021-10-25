#include "stereo_streamer/stereo_publisher_node.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include <opencv2/highgui/highgui.hpp>


StereoPublisherNode::StereoPublisherNode() : Node("stereoPublisherNode")
{
  // declare parameters 
  this->declare_parameter("video_device", 0);
  this->declare_parameter("left_camera_info", "");
  this->declare_parameter("right_camera_info", "");

  // load parameters 
  this->videoDevice = this->get_parameter("video_device").as_int();
  this->leftCameraInfoPath = this->get_parameter("left_camera_info").as_string();
  this->rightCameraInfoPath = this->get_parameter("right_camera_info").as_string();

  // connect to video device 
  if(!connect_to_video_device())
    return;

  // load camera calibration data
  load_camera_info();

  // run publisher 
  run_node();
}

bool StereoPublisherNode::connect_to_video_device()
{
  RCLCPP_INFO(get_logger(), "connecting to /dev/video%d", this->videoDevice);

  imageStream.open(this->videoDevice);

  rclcpp::Time next_stamp_ = rclcpp::Clock().now();

  if(!imageStream.isOpened()){
    RCLCPP_ERROR(get_logger(), "failed to connect to /dev/video%d", this->videoDevice);
    return false;
  }

  RCLCPP_INFO(get_logger(), "connected to /dev/video%d", this->videoDevice);

  return true;
}

bool StereoPublisherNode::load_camera_info()
{
  std::string camera_name;

  bool leftInfoLoaded = false, rightInfoLoaded = false;

  if (!this->leftCameraInfoPath.empty() && camera_calibration_parsers::readCalibration(this->leftCameraInfoPath, camera_name, leftInfo)) {
      RCLCPP_INFO(get_logger(), "loaded left camera calibration data");

      this->leftInfo.header.frame_id = "left_camera_information";

      leftInfoLoaded = true;
    } else 
        RCLCPP_INFO(get_logger(), "no left calibration data found");

  if (!this->rightCameraInfoPath.empty() && camera_calibration_parsers::readCalibration(this->rightCameraInfoPath, camera_name, rightInfo)) {
      RCLCPP_INFO(get_logger(), "loaded right camera calibration data");

      this->rightInfo.header.frame_id = "right_camera_information";

      rightInfoLoaded = true;
    } else 
        RCLCPP_INFO(get_logger(), "no right calibration data found");

  return leftInfoLoaded & rightInfoLoaded;
}

std::string StereoPublisherNode::mat_type2encoding(const int& mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";

    case CV_8UC3:
      return "bgr8";

    case CV_16SC1:
      return "mono16";

    case CV_8UC4:
      return "rgba8";

    default:
      throw std::runtime_error("unsupported encoding type");
  }
}

void StereoPublisherNode::run_node()
{
  RCLCPP_INFO(get_logger(), "started publishing");

  while (rclcpp::ok()) {
    cv::Mat combinedFrame, leftFrame, rightFrame;

    sensor_msgs::msg::Image leftImage, rightImage;

    imageStream >> combinedFrame;

    if(!combinedFrame.empty()) {

      leftFrame = combinedFrame(cv::Range(0, combinedFrame.rows), cv::Range(combinedFrame.cols / 2, combinedFrame.cols)).clone();
      rightFrame = combinedFrame(cv::Range(0, combinedFrame.rows), cv::Range(0, combinedFrame.cols / 2)).clone();

      cv::flip(leftFrame, leftFrame, -1);
      cv::flip(rightFrame, rightFrame, -1);
      
      rclcpp::Time stamp = rclcpp::Clock().now();
      leftImage.header.stamp = stamp;
      leftImage.header.frame_id = "camera_frame";
      leftImage.height = leftFrame.size[0];
      leftImage.width = leftFrame.size[1];;
      leftImage.encoding = mat_type2encoding(leftFrame.type());;
      leftImage.is_bigendian = false;
      leftImage.step = static_cast<sensor_msgs::msg::Image::_step_type>(leftFrame.step);
      leftImage.data.assign(leftFrame.datastart, leftFrame.dataend);

      rightImage.header.stamp = stamp;
      rightImage.header.frame_id = "camera_frame";
      rightImage.height = rightFrame.size[0];
      rightImage.width = rightFrame.size[1];;
      rightImage.encoding = mat_type2encoding(rightFrame.type());
      rightImage.is_bigendian = false;
      rightImage.step = static_cast<sensor_msgs::msg::Image::_step_type>(rightFrame.step);
      rightImage.data.assign(rightFrame.datastart, rightFrame.dataend);

      leftInfo.header.stamp = stamp;
      rightInfo.header.stamp = stamp;

      this->stereoPublisher.publishInfo(leftInfo, rightInfo);
      this->stereoPublisher.publishImages(leftImage, rightImage);
    }
  }
}

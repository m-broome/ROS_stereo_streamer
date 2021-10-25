#include "stereo_streamer/stereo_publisher.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


StereoPublisher::StereoPublisher() : Node("stereoPublisher")
{
  this->leftImagePublisher = create_publisher<sensor_msgs::msg::Image>("image_raw_l", 10);

  this->leftInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_l", 10);

  this->rightImagePublisher = create_publisher<sensor_msgs::msg::Image>("image_raw_r", 10);

  this->rightInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_r", 10);
}

void StereoPublisher::publishImages(const sensor_msgs::msg::Image& leftImg, const sensor_msgs::msg::Image& rightImg)
{
  this->leftImagePublisher->publish(leftImg);

  this->rightImagePublisher->publish(rightImg);
}

void StereoPublisher::publishInfo(const sensor_msgs::msg::CameraInfo& leftInfo, const sensor_msgs::msg::CameraInfo& rightInfo)
{
  this->leftInfoPublisher->publish(leftInfo);
  
  this->rightInfoPublisher->publish(rightInfo);
}

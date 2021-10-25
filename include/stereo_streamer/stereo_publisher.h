#ifndef STEREO_PUBLISHER_H
#define STEREO_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


class StereoPublisher : public rclcpp::Node
{
  public:

    StereoPublisher();

    void publishImages(const sensor_msgs::msg::Image& leftImg, const sensor_msgs::msg::Image& rightImg);

    void publishInfo(const sensor_msgs::msg::CameraInfo& leftInfo, const sensor_msgs::msg::CameraInfo& rightInfo);

  private:

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftImagePublisher;
    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoPublisher;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightImagePublisher;
    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoPublisher;
};

#endif
#ifndef STEREO_PUBLISHER_NODE_H
#define STEREO_PUBLISHER_NODE_H

#include "stereo_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/highgui/highgui.hpp>


class StereoPublisherNode : public rclcpp::Node{

  public:

    StereoPublisherNode();

  private:

    int videoDevice;

    std::string leftCameraInfoPath, rightCameraInfoPath;

    StereoPublisher stereoPublisher;

    sensor_msgs::msg::CameraInfo leftInfo, rightInfo;

    cv::VideoCapture imageStream;

    std::string mat_type2encoding(const int& mat_type);

    bool connect_to_video_device();

    bool load_camera_info();

    void run_node();
};

#endif
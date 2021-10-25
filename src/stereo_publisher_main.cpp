#include "stereo_streamer/stereo_publisher_node.h"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  StereoPublisherNode StereoPublisherNode;

  return 0;
}
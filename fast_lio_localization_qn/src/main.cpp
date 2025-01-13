#include "fast_lio_localization_qn.h"
#include <rclcpp/multi_threaded_executor.hpp>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("fast_lio_localization_qn");

  FastLioLocalizationQn FastLioLocalizationQn_(nh_private);

  // ros::AsyncSpinner spinner(3); // Use multi threads
  // spinner.start();
  // ros::waitForShutdown();

    return 0;
}

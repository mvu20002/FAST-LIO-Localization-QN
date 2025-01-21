#include "fast_lio_localization_qn.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("fast_lio_localization_qn");

  FastLioLocalizationQn FastLioLocalizationQn_(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}

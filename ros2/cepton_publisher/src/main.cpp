#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cepton_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Optionally use an executer to load nodes at runtime.
  // Can also pass the node directly to rclcpp::spin
  rclcpp::executors::SingleThreadedExecutor exec;

  // Add the publisher
  auto publisher = make_shared<cepton_ros::CeptonPublisher>();

  exec.add_node(publisher);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
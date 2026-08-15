#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "acadbot_courier/courier_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    // Single-threaded on purpose. Every callback in CourierServer returns
    // promptly and none of them blocks, so serialising them is enough to make
    // the node's state safe without a single mutex.
    rclcpp::spin(std::make_shared<acadbot_courier::CourierServer>());
  } catch (const std::exception & e) {
    // The location table refuses to start on a bad floor plan, and its
    // messages are written to be read. Caught here so a configuration mistake
    // prints as one fatal line rather than as an abort trace.
    RCLCPP_FATAL(rclcpp::get_logger("courier_server"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

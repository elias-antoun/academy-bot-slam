#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "acadbot_courier/courier_bt_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<acadbot_courier::CourierBtServer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("courier_bt_server"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

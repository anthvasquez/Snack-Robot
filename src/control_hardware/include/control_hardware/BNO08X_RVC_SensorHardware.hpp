
#ifndef BNO08X_RVC_SENSORHARDWARE_HPP
#define BNO08X_RVC_SENSORHARDWARE_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "visibility_control.h"

namespace control_hardware
{
  class BNO08X_RVC_SensorHardware : public hardware_interface::SensorInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BNO08X_RVC_SensorHardware)

    SNACK_ROBOT_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    SNACK_ROBOT_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    sensor_msgs::msg::Imu imu;
    int uart_fd;
  };
} // namespace control_hardware
#endif // BNO08X_RVC_SENSORHARDWARE_HPP
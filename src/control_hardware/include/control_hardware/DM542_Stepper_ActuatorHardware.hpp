
#ifndef DM542_STEPPER_ACTUATORHARDWARE_HPP
#define DM542_STEPPER_ACTUATORHARDWARE_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace control_hardware
{
    class DM542_Stepper_ActuatorHardware : public hardware_interface::ActuatorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DM542_Stepper_ActuatorHardware)

        SNACK_ROBOT_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        SNACK_ROBOT_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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

        SNACK_ROBOT_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        double state_position;  //position of shaft out of # of pulses per revolution
        double state_velocity;  // m/s
        double cmd_vel;         // m/s
        float cmd_pulse_per_second;
        int pul_pin, en_pin, dir_pin;
        int pulse_per_rev;
        int handle;
        int max_pulse_freq = 5000;  // Hz
    };
} // namespace control_hardware
#endif // DM542_STEPPER_ACTUATORHARDWARE_HPP
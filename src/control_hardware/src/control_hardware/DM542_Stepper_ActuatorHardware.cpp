#include "control_hardware/DM542_Stepper_ActuatorHardware.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <lgpio.h>
#include <cmath>

using namespace std;
using namespace hardware_interface;

static const rclcpp::Logger logger = logger;
const double wheel_radius = 0.095;
const double wheel_circumference = 2 * M_PI * wheel_radius;

namespace control_hardware
{
    static bool StateInterfaceExists(vector<InterfaceInfo> states, string interfaceName)
    {
        for(auto state : states)
        {
            if(state.name == interfaceName)
            {
                return true;
            }
        }
        return false;
    }

    vector<StateInterface> DM542_Stepper_ActuatorHardware::export_state_interfaces()
    {
        vector<StateInterface> state_interfaces;
        state_interfaces.push_back(StateInterface(info_.joints[0].name, HW_IF_POSITION, &state_position));
        state_interfaces.push_back(StateInterface(info_.joints[0].name, HW_IF_VELOCITY, &state_velocity));

        return state_interfaces;
    }

    vector<CommandInterface> DM542_Stepper_ActuatorHardware::export_command_interfaces()
    {
        vector<CommandInterface> command_interfaces;
        command_interfaces.push_back(CommandInterface(info_.joints[0].name, HW_IF_VELOCITY, &cmd_vel));

        return command_interfaces;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_init(const HardwareInfo &info)
    {
        if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        if (info_.joints.size() != 1)
        {
            RCLCPP_FATAL(logger, "Wrong number of joints specified.  Expected: 1, Actual: %d", static_cast<int>(info_.joints.size()));
            return CallbackReturn::ERROR;
        }

        auto joint = info_.joints[0];
        auto commands = joint.command_interfaces;
        auto states = joint.state_interfaces;

        if (joint.name.empty())
        {
            RCLCPP_FATAL(logger, "ros2_control tag %s does not have a joint name.", info_.name.c_str());
            return CallbackReturn::ERROR;
        }

        if (commands.size() != 1 || commands[0].name != HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(logger, "joint %s must specify exactly one velocity interface", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        if (states.size() != 2)
        {
            RCLCPP_FATAL(logger, "joint %s must specify exactly 2 state interfaces", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        if (StateInterfaceExists(states, HW_IF_POSITION) || StateInterfaceExists(states, HW_IF_VELOCITY))
        {
            RCLCPP_FATAL(logger, "joint %s must specify one position state and one velocity state.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters["en_pin"].empty())
        {
            RCLCPP_FATAL(logger, "joint %s must specify an 'en_pin' param.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (info_.hardware_parameters["pul_pin"].empty())
        {
            RCLCPP_FATAL(logger, "joint %s must specify an 'pul_pin' param.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (info_.hardware_parameters["dir_pin"].empty())
        {
            RCLCPP_FATAL(logger, "joint %s must specify an 'dir_pin' param.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if(info_.hardware_parameters["pulse_per_rev"].empty())
        {
            RCLCPP_FATAL(logger, "joint %s must specify an 'pulse_per_rev' param.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        try
        {
            auto joint = info_.joints[0];
            state_position = 0;
            state_velocity = 0;
            cmd_vel = 0;

            pul_pin = stoi(info_.hardware_parameters["pul_pin"]);
            dir_pin = stoi(info_.hardware_parameters["dir_pin"]);
            en_pin = stoi(info_.hardware_parameters["en_pin"]);
            pulse_per_rev = stof(info_.hardware_parameters["pulse_per_rev"]);

            handle = lgGpiochipOpen(0);
            if (handle < 0)
            {
                RCLCPP_FATAL(logger, "Could not get handle to gpiochip0 with error code %d", handle);
                return CallbackReturn::ERROR;
            }
        }
        catch (const invalid_argument &e)
        {
            RCLCPP_INFO(logger, "en_pin: %s, pul_pin: %s, dir_pin: %s, pulse_per_rev: %s",
                        info_.hardware_parameters["en_pin"].c_str(),
                        info_.hardware_parameters["pul_pin"].c_str(),
                        info_.hardware_parameters["dir_pin"].c_str(),
                        info_.hardware_parameters["pulse_per_rev"].c_str()
                    );
            RCLCPP_FATAL(logger, "Could not parse one or more values from %s hardware parameters.", info_.name.c_str());
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        state_position = 0;
        state_velocity = 0;
        cmd_vel = 0;
        cmd_pulse_per_second = 0;
        handle = lgGpiochipClose(0);
        if (handle < 0)
        {
            RCLCPP_FATAL(logger, "Could not close gpiochip0 with error code %d", handle);
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        if(lgGpioClaimOutput(handle, 0, en_pin, 1) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to claim gpio %d as output pin.", en_pin);
            return CallbackReturn::FAILURE;
        }
        if(lgGpioClaimOutput(handle, 0, pul_pin, 0) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to claim gpio %d as output pin.", pul_pin);
            return CallbackReturn::FAILURE;
        }
        if(lgGpioClaimOutput(handle, 0, dir_pin, 1) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to claim gpio %d as output pin.", dir_pin);
            return CallbackReturn::FAILURE;
        }

        state_position = 0;
        state_velocity = 0;
        cmd_vel = 0;
        cmd_pulse_per_second = 0;

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        if(lgGpioWrite(handle, en_pin, 0) != 0)
        {
            RCLCPP_ERROR(logger, "Could not set 'en_pin' to low state.");
        }

        if(lgGpioFree(handle, en_pin) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to free gpio pin %d", en_pin);
            return CallbackReturn::FAILURE;
        }
        if(lgGpioFree(handle, pul_pin) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to free gpio pin %d", pul_pin);
            return CallbackReturn::FAILURE;
        }
        if(lgGpioFree(handle, dir_pin) != 0)
        {
            RCLCPP_FATAL(logger, "Failed to free gpio pin %d", dir_pin);
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DM542_Stepper_ActuatorHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    return_type DM542_Stepper_ActuatorHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        auto revs_per_second = cmd_vel / wheel_circumference;
        cmd_pulse_per_second = revs_per_second * pulse_per_rev;

        if(lgGpioWrite(handle, dir_pin, cmd_vel < 0 ? 0 : 1) != 0)
        {
            RCLCPP_ERROR(logger, "Could not write to 'dir_pin' for joint %s", info_.joints[0].name.c_str());
            return return_type::ERROR;
        }
        if(lgTxPwm(handle, pul_pin, abs(cmd_pulse_per_second), 50, 0, 0) < 0)
        {
            RCLCPP_ERROR(logger, "Failed to create software pwm signal to 'pul_pin' on joint %s", info_.joints[0].name.c_str());
            return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type DM542_Stepper_ActuatorHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // cmd_pulse_per_second = cmd_vel / wheel_circumference * pulse_per_rev;
        state_velocity = (cmd_pulse_per_second / pulse_per_rev) * wheel_circumference;
        auto pulses_traveled = static_cast<int>(period.nanoseconds() * cmd_pulse_per_second / pow(10, 9)) + 1;
        state_position = ((int)state_position + pulses_traveled) % pulse_per_rev;

        return return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_hardware::DM542_Stepper_ActuatorHardware, hardware_interface::ActuatorInterface)

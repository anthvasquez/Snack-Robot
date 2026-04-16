#include "control_hardware/BNO08X_RVC_SensorHardware.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <lgpio.h>
#include <vector>
#include <string>
#include <termios.h>
#include <cmath>
#include <stdint.h>

using namespace std;
using namespace hardware_interface;

namespace control_hardware
{
  static const rclcpp::Logger logger = rclcpp::get_logger("BNO08X_IMU");
  const char SERIAL_DEVICE[20] = "/dev/serial0";
  char buffer[200];
  const int READING_SIZE = 19;
  const double DEG_TO_RAD = M_PI / 180;

#pragma region Inactive State Transitions

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if(SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    if (info_.sensors.size() != 1)
    {
      RCLCPP_ERROR(logger, "Expected exactly 1 sensor but %d were found.", (int)info_.sensors.size());
      return CallbackReturn::ERROR;
    }
    if (info_.sensors[0].name.empty())
    {
      RCLCPP_ERROR(logger, "Sensor must specify a name.");
      return CallbackReturn::ERROR;
    }

    if (info_.sensors[0].state_interfaces.size() != 10)
    {
      RCLCPP_ERROR(logger, "Wrong number of state interfaces specified.  Expected: 10, Actual: %d", (int)info_.sensors[0].state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    (void) previous_state;
    return CallbackReturn::SUCCESS;
  }

#pragma endregion

#pragma region Configured State Transitions

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    (void) previous_state;
    uart_fd = lgSerialOpen(SERIAL_DEVICE, 115200, 0);
    if (uart_fd < 0)
    {
      RCLCPP_ERROR(logger, "Could not get device handle for %s: %d", SERIAL_DEVICE, uart_fd);
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state)
  {
    (void) previous_state;
    auto success = lgSerialClose(uart_fd);
    if (success < 0)
    {
      RCLCPP_ERROR(logger, "Failed to close serial device with error code %d.", success);
    }

    return CallbackReturn::SUCCESS;
  }

#pragma endregion

#pragma region Active State Transitions

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    (void) previous_state;
    imu = sensor_msgs::msg::Imu(); // reset imu message values to zeros
    tcflush(uart_fd, TCIOFLUSH);   // flush any stale data from the buffer
    // This IMU doesn't report angular velocity info, so tell programs to ignore this field
    imu.angular_velocity_covariance[0] = -1;

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BNO08X_RVC_SensorHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    (void) previous_state;
    return CallbackReturn::SUCCESS;
  }

#pragma endregion

  std::vector<hardware_interface::StateInterface> BNO08X_RVC_SensorHardware::export_state_interfaces()
  {
    vector<StateInterface> state_interfaces;
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "orientation.x", &imu.orientation.x));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "orientation.y", &imu.orientation.y));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "orientation.z", &imu.orientation.z));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "orientation.w", &imu.orientation.w));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "angular_velocity.x", &imu.angular_velocity.x));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "angular_velocity.y", &imu.angular_velocity.y));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "angular_velocity.z", &imu.angular_velocity.z));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "linear_acceleration.x", &imu.linear_acceleration.x));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "linear_acceleration.y", &imu.linear_acceleration.y));
    state_interfaces.push_back(StateInterface(info_.sensors[0].name, "linear_acceleration.z", &imu.linear_acceleration.z));

    return state_interfaces;
  }

  hardware_interface::return_type BNO08X_RVC_SensorHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void) time;
    (void) period;
    auto dataAvailable = lgSerialDataAvailable(uart_fd);
    if (dataAvailable >= READING_SIZE)
    {
      auto bytesRead = dataAvailable >= READING_SIZE * 2 ? lgSerialRead(uart_fd, buffer, READING_SIZE * 2) : lgSerialRead(uart_fd, buffer, READING_SIZE);
      if (bytesRead < READING_SIZE)
      {
        RCLCPP_WARN(logger, "Failed to read all data available (%d)", bytesRead);
        RCLCPP_INFO(logger, "-------------------------");
        return return_type::ERROR;
      }

      char *reading = NULL;
      for (int i = bytesRead - READING_SIZE; i >= 0; i--)
      {
        if (buffer[i] == (char)0xAA && buffer[i + 1] == (char)0xAA)
        {
          reading = buffer + i + 2;
          break;
        }
      }
      if (reading == nullptr)
      {
        RCLCPP_WARN(logger, "Reading (%d) did not contain 0xAAAA header with enough space for the full message", bytesRead);
        RCLCPP_INFO(logger, "-----------------------");
        stringstream ss;
        for (int i = 0; i < bytesRead; i++)
        {
          ss << hex << uppercase << static_cast<int>(buffer[i]);
        }
        RCLCPP_DEBUG(logger, ss.str().c_str());
        return return_type::OK;
      }

      auto sequence = reading[0];
      //convert centidegrees to rad
      auto yaw = (int16_t)(reading[1] + (reading[2] << 8)) * 0.01;
      auto pitch = (int16_t)(reading[3] + (reading[4] << 8)) * 0.01;
      auto roll = (int16_t)(reading[5] + (reading[6] << 8)) * 0.01;
      tf2::Quaternion orientation;
      orientation.setRPY(roll * DEG_TO_RAD, pitch * DEG_TO_RAD, yaw * DEG_TO_RAD);
      orientation.normalize();
      tf2::convert(orientation, imu.orientation);
      
      imu.linear_acceleration.x = (int16_t)(reading[7] + (reading[8] << 8)) * 0.01;
      imu.linear_acceleration.y = (int16_t)(reading[9] + (reading[10] << 8)) * 0.01;
      imu.linear_acceleration.z = (int16_t)(reading[11] + (reading[12] << 8)) * 0.01;

      RCLCPP_DEBUG(logger, "Index: %d", sequence);
      RCLCPP_DEBUG(logger, "Yaw: %.2f, Pitch: %.2f, Roll: %.2f", imu.angular_velocity.z, imu.angular_velocity.y, imu.angular_velocity.x);
      RCLCPP_DEBUG(logger, "Acceleration X: %.2f, Y: %.2f, Z: %.2f m/s^2", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);

      return return_type::OK;
    }
    else
    {
      RCLCPP_WARN(logger, "data did not contain enough bytes for a full message (%d).  Consider decreasing read frequency.", dataAvailable);
      return return_type::OK;
    }
  }

} // namespace snack_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_hardware::BNO08X_RVC_SensorHardware, hardware_interface::SensorInterface)

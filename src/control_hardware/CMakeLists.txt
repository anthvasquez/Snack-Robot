cmake_minimum_required(VERSION 3.8)
project(control_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_lifecycle
  ros2_control
  pluginlib
  hardware_interface
  lifecycle_msgs
  sensor_msgs
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  sh2
  SHARED
  src/sh2/sh2.c
  src/sh2/euler.c
  src/sh2/sh2_SensorValue.c
  src/sh2/sh2_util.c
  src/sh2/shtp.c
)
target_include_directories(sh2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/sh2>
$<INSTALL_INTERFACE:include/sh2>
)

add_library(
  BNO08X_IMU
  SHARED
  src/control_hardware/BNO08X_RVC_SensorHardware.cpp
)
target_compile_features(BNO08X_IMU PUBLIC cxx_std_17)
target_include_directories(BNO08X_IMU PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/control_hardware>
)
target_link_libraries(BNO08X_IMU PUBLIC -llgpio sh2)
ament_target_dependencies(
  BNO08X_IMU PUBLIC
  ${DEPENDENCIES}
)

add_library(
  DM542_Stepper
  SHARED
  src/control_hardware/DM542_Stepper_ActuatorHardware.cpp
)
target_compile_features(DM542_Stepper PUBLIC cxx_std_17)
target_include_directories(DM542_Stepper PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/control_hardware>
)
target_link_libraries(DM542_Stepper PUBLIC -llgpio)
ament_target_dependencies(
  DM542_Stepper PUBLIC
  ${DEPENDENCIES}
)

install(TARGETS
  BNO08X_IMU
  DM542_Stepper
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(hardware_interface control_hardware.xml)

install(DIRECTORY
	launch
	config
  urdf
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()

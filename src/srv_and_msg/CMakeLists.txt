cmake_minimum_required(VERSION 3.8)
project(srv_and_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Add msgs/srvs
set(msg_files
  "msg/OdriveStatus.msg"
)
set(srv_files
  "srv/ClearErrors.srv"
  "srv/GetEncoderCount.srv"
  "srv/GetEncoderError.srv"
  "srv/GetEncoderEstimates.srv"
  "srv/GetIq.srv"
  "srv/GetMotorError.srv"
  "srv/GetTemperature.srv"
  "srv/GetVbusVoltage.srv"
  "srv/OdriveEstop.srv"
  "srv/ResetOdrive.srv"
  "srv/SetAbsolutePos.srv"
  "srv/SetAxisNodeId.srv"
  "srv/SetAxisRequestedState.srv"
  "srv/SetAxisStartupConfig.srv"
  "srv/SetControllerModes.srv"
  "srv/SetInputPos.srv"
  "srv/SetInputVel.srv"
  "srv/SetPosGain.srv"
  "srv/SetTrajAccelLimits.srv"
  "srv/SetTrajInertia.srv"
  "srv/SetTrajVelLimit.srv"
  "srv/SetVelGains.srv"
  "srv/SetVelLimit.srv"
  "srv/StartAnticogging.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
)

# Allows the use of msgs/srvs defined in this package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_get_typesupport_target(${PROJECT_NAME}_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
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


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()

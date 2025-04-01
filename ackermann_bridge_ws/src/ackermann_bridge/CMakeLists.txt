cmake_minimum_required(VERSION 3.8)
project(ackermann_bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(ackermann_bridge)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install systemd service
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch wrapper
install(
  PROGRAMS launch/bridge_wrapper.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Post-install script for systemd
if(UNIX AND NOT APPLE)
  install(CODE [[
    execute_process(
      COMMAND bash "-c" 
        "mkdir -p /etc/systemd/system/ && 
         cp ${CMAKE_INSTALL_PREFIX}/share/ackermann_bridge/config/ackermann_bridge.service /etc/systemd/system/ &&
         systemctl daemon-reload"
      RESULT_VARIABLE result
    )
    if(NOT result EQUAL 0)
      message(FATAL_ERROR "Failed to install systemd service")
    endif()
  ]])
endif()

ament_package()

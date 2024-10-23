cmake_minimum_required(VERSION 3.8)
project(denso_robot_bringup)

# Add export for compile commands for clangd and other development tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(turtle_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()

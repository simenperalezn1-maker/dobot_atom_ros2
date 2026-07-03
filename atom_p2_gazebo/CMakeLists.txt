cmake_minimum_required(VERSION 3.8)
project(atom_p2_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

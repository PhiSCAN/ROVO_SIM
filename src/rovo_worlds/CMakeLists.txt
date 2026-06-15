cmake_minimum_required(VERSION 3.8)
project(rovo_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

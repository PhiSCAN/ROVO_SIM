cmake_minimum_required(VERSION 3.8)
project(rovo_gz_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    scripts/depth_float32_to_uint16.py
    scripts/aruco_bot_movement.py
    scripts/velodyne_time_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Make package discoverable to gz-sim as a model location
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

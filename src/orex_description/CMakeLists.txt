cmake_minimum_required(VERSION 3.8)
project(orex_description)

# Find ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Install static directories containing launch scripts, meshes, and URDF files
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(phoenix_drone_gazebo)

find_package(ament_cmake REQUIRED)

# Install world files
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config (ground-truth manifest, etc.)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz configs
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install the world generator script
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

ament_package()

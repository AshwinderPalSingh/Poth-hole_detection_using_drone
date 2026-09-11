cmake_minimum_required(VERSION 3.8)
project(phoenix_drone_description)

find_package(ament_cmake REQUIRED)

# ──────────────────────────────────────────────────────────
# Install the Gazebo model into the share directory so that
# both Gazebo and other ROS 2 packages can locate it via
# GZ_SIM_RESOURCE_PATH / GAZEBO_MODEL_PATH.
# ──────────────────────────────────────────────────────────
install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
    urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Expose the model path via an environment hook so that
# sourcing the workspace automatically adds our models.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(odometry)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelInfo.msg"
  "msg/WheelRPMs.msg"
  "msg/DirectionAngles.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
install(
  DIRECTORY odometry
  DESTINATION lib/python3.10/site-packages
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


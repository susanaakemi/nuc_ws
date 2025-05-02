cmake_minimum_required(VERSION 3.5)
project(odometry)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf_transformations REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelInfo.msg"
  "msg/WheelRPMs.msg"
  "msg/DirectionAngles.msg"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY odometry
  DESTINATION lib/python3.10/site-packages
)

ament_python_install_package(${PROJECT_NAME})

ament_package()

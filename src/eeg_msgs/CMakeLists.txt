cmake_minimum_required(VERSION 3.8)
project(eeg_msgs)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate the message interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EEGBlock.msg"
  DEPENDENCIES std_msgs
)

# Export the package
ament_package()

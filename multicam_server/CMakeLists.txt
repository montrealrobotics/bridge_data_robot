cmake_minimum_required(VERSION 3.8)
project(multicam_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  multicam_server/start_streamers.py
  multicam_server/streamer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

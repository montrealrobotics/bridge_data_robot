cmake_minimum_required(VERSION 3.8)
project(widowx_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GotoNeutral.srv"
  "srv/OpenGripper.srv"
  "srv/MoveToEEP.srv"
  "srv/MoveToState.srv"
  "srv/GetGripperDesiredState.srv"
  "srv/GetCartesianPose.srv"
  "srv/GetState.srv"
  "srv/GetVRButtons.srv"
  "srv/EnableController.srv"
  "srv/DisableController.srv"
  "srv/SetGripperPosition.srv"
  DEPENDENCIES std_msgs
)

# ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  # Add your Python executable scripts here
  # Example: scripts/my_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other directories (config, etc.)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
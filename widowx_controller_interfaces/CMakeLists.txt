cmake_minimum_required(VERSION 3.8)
project(widowx_controller_interfaces)

find_package(ament_cmake REQUIRED)
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

ament_package()


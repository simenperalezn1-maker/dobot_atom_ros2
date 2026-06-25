cmake_minimum_required(VERSION 3.5)
project(dobot_atom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AMRBasicStatus.msg"
  "msg/AMRCommand.msg"
  "msg/AMREventStatus.msg"
  "msg/AMRState.msg"
  "msg/AxisStateInfo.msg"
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/BmsInfo.msg"
  "msg/CalibrateImu.msg"
  "msg/CerebrumCmd.msg"
  "msg/CerebrumStatus.msg"
  "msg/ChangeMode.msg"
  "msg/ClearErrors.msg"
  "msg/DHGripperSingleCmd.msg"
  "msg/DHGripperSingleState.msg"
  "msg/DHGripperCmd.msg"
  "msg/DHGripperState.msg"
  "msg/EcatSlaveInfo.msg"
  "msg/EmergencyState.msg"
  "msg/EnableMotors.msg"
  "msg/GoHome.msg"
  "msg/HandsCmd.msg"
  "msg/HandsState.msg"
  "msg/IMUState.msg"
  "msg/InspireCmd.msg"
  "msg/InspireState.msg"
  "msg/JoystickValue.msg"
  "msg/LaserScan.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/LowerCmd.msg"
  "msg/LowerState.msg"
  "msg/MainNodesState.msg"
  "msg/ModifyJointOffset.msg"
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/Odom.msg"
  "msg/PlayTTS.msg"
  "msg/PowerCmd.msg"
  "msg/PowerState.msg"
  "msg/RemoteControl.msg"
  "msg/SetCalibrationFlag.msg"
  "msg/SetFsmId.msg"
  "msg/SwitchUpperControl.msg"
  "msg/TeleopCommand.msg"
  "msg/TeleopStatus.msg"
  "msg/UpperCmd.msg"
  "msg/UpperState.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_generate_dds_interfaces(
  ${rosidl_generate_interfaces_TARGET}__dds_connext_idl
  IDL_TUPLES ${rosidl_generate_interfaces_IDL_TUPLES}
  OUTPUT_SUBFOLDERS "dds_connext"
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}__dds_connext_idl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

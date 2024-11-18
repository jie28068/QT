cmake_minimum_required(VERSION 2.8)
project(hmi_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg/ControlCommand
  FILES
  ControlCommand.msg
)

add_message_files(
  DIRECTORY msg/HmiStatus
  FILES
  AutoModeStatus.msg
  LivoxConnectionStatus.msg
  PLCStopSignalStatus.msg
  PLCConnectionStatus.msg
  PneumaticPickStatus.msg
  RobotConnectionStatus.msg
  RobotControllerStatus.msg
  RunModeStatus.msg
  SafetyDoorsEnableStatus.msg
  SafetyDoorStatus.msg
  TaskEnableState.msg
)

add_message_files(
  DIRECTORY msg
  FILES
  ManualCommand.msg
  HmiStatus.msg
  OperationLogInfo.msg
  SystemLogInfo.msg
  SystemParamsSrvType.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ControlCommandSrv.srv
  QueryOperationLogsSrv.srv
  QuerySystemLogsSrv.srv
  SystemParamsSrv.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  # LIBRARIES
  CATKIN_DEPENDS roscpp message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

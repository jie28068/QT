/**
 * @file hmi_datatypes.h
 * @author Zhao Xingyu (zhaoxingyu@wattman.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-19
 * 
 * Copyright (c) 2024, Wattman, Inc. All Rights Reserved.
 * 
 */

#ifndef WATTMAN_AUTO_ARM_HMI_DATATYPES_H
#define WATTMAN_AUTO_ARM_HMI_DATATYPES_H

#include <map>
#include <string>

namespace auto_arm {
namespace task {

enum class HmiSystemStatusKey : int {
  RobotStatus = 0,
  LivoxStatus = 1,
  CameraStatus = 2,
  PLCStatus = 3,
  AirPressure = 6,
  WorkingCount = 8,
  PneumaticPick = 13,
  SafetyDoor_1 = 14,
  SafetyDoor_2 = 15,
  RoomTemperature = 18,
};

static std::map<HmiSystemStatusKey, std::string> system_status_desc = {
    {HmiSystemStatusKey::RobotStatus, "机械臂连接状态"},
    {HmiSystemStatusKey::LivoxStatus, "激光雷达连接状态"},
    {HmiSystemStatusKey::CameraStatus, "相机连接状态"},
    {HmiSystemStatusKey::PLCStatus, "PLC连接状态"},
    {HmiSystemStatusKey::AirPressure, "风镐气压值"},
    {HmiSystemStatusKey::WorkingCount, "清渣次数"},
    {HmiSystemStatusKey::PneumaticPick, "风镐开关"},
    {HmiSystemStatusKey::SafetyDoor_1, "安全门_1"},
    {HmiSystemStatusKey::SafetyDoor_2, "安全门_2"},
    {HmiSystemStatusKey::RoomTemperature, "控制柜内部温度"},
};

enum class RobotStatus :bool {
  ONLINE = true,
  OFFLINE = false
};

}  // namespace task

}  // namespace auto_arm

#endif  //  WATTMAN_AUTO_ARM_HMI_DATATYPES_H
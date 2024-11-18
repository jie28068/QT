# HMI Status Message
AutoModeStatus auto_mode_status          
RobotConnectionStatus robot_connection_status
PLCStopSignalStatus plc_stop_signal_staus        # 暂停信号来源反馈
PLCConnectionStatus plc_connection_status
LivoxConnectionStatus livox1_connection_status   # 东入料口激光雷达连接状态
LivoxConnectionStatus livox2_connection_status   # 西入料口激光雷达连接状态
PneumaticPickStatus pneumatic_pick_status
RobotControllerStatus robot_controller_status    # 机器人控制器状态
TaskEnableState clean_hole1_enable_state
TaskEnableState clean_hole2_enable_state
RunModeStatus run_mode   # 系统运行状态 1: 就地, 2: 远程, 3: 停止(未定义）, 4: 故障(未定义）
int32 clean_count  # 清渣作业次数
SafetyDoorStatus safety_door_1  # 安全门1状态, 3: 未关闭, 4： 未锁定, 5: 已锁定 
SafetyDoorStatus safety_door_2  # 安全门2状态, 3: 未关闭, 4： 未锁定, 5: 已锁定 
SafetyDoorsEnableStatus safety_door_enable_status  # 安全门使能状态, 3: 禁用, 4: 启用
float32 cabinet_temperature  # 机柜温度
float32 pneumatic_pick_pressure # 风镐气压
float32 cooler_air_pressure    # 冷却气压
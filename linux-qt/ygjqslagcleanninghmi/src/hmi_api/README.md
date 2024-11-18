# 豫光金铅通渣机器人通信API

## Topic订阅

1. 系统运行状态
    - 主题: `/hmi/status`
    - 消息类型: `hmi_msgs/HmiStatus`
    - 消息内容: 机器人系统运行状态

## Service主题

1. 机器人控制
    - 主题: `/hmi/control_command`
    - 消息类型: `hmi_msgs/ControlCommandSrv`
    - 消息内容: 控制指令

2. 系统日志
    - 主题: `/hmi/query_system_logs`
    - 消息类型: `hmi_msgs/QuerySystemLogsSrv`
    - 消息内容: 日志内容

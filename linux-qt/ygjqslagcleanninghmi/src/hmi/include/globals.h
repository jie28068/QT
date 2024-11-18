#ifndef GLOBALS_H
#define GLOBALS_H

#include <QString>
#include <QMap>

namespace globals
{
    /// @brief  对话框类型
    enum DialogType
    {
        WarningDialog = 0, // 错误警告
        ConfirmDialog = 1  // 消息确认
    };

    static const QString toBeDetected = "待检测";
    static const QString offline = "离线";
    static const QString online = "在线";
    static const QString closed = "关闭";
    static const QString on = "开启";
    static const QString emergencyStop = "急停";
    static const QString abnormal = "异常";
    static const QString dropped = "未知";
    static const QString automaticMode = "自动模式";
    static const QString manualMode = "手动模式";
    static const QString systemLog = "系统日志";
    static const QString operationLog = "操作日志";
    static const QString droppedDo = "不可用";
    static const QString normalMessage = "消息";
    static const QString warningMessage = "警告";
    static const QString errorMessage = "错误";
    static const QMap<int, QString> MessageTypeMap = {{0, normalMessage}, {1, warningMessage}, {2, errorMessage}};
    static const QString workTimeUpdate = "工作时间间隔更新";
    static const QString cleanUpdate = "清渣使能更新";
    static const QString toolOffsetUpdate = "工具偏移量更新";
    static const QString systemParamsLoad = "读取";
    static const QString notclosed = "未关闭";
    static const QString unlocked = "未上锁";
    static const QString islocked = "已上锁";
    static const QString disable = "禁用";
    static const QString enable = "启用";
    static const QString stop = "已停止";
    static const QString running = "正在运行";
}

#endif
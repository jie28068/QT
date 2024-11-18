#pragma once

#include <QString>

namespace JsonDefines
{
    enum HMI
    {
        UPLOAD_TRAJ = 0,     // 上传已生成轨迹
        UPLOAD_TEMPLATE = 1, // 上传轨迹生成模板
        CREATE_TRAJ = 2,     // 生成轨迹
        UPLOAD_PARAMS = 3,   // 上传参数
        UPDATA_PARAMS = 4,   // 更新参数
        ASK_FOR_CARBON = 5,  // 请求炭块
        PLC_STATUS = 6,      // PLC系统运行
        LINE_STATUS = 7,     // 线路清理状态
        BACK_HOME = 8,       // 返回原位
        CHECK_KNIFE = 9,     // 前往检修位
        TRAJ_SPEED = 10,     // 轨迹速度调整
        CAMERA_CAL = 11,     // 标定数据计算
        CN_LIST = 12,        // 上传中文注释表
    };

    static const QString TRACKTYPE = "type";    // 轨迹类型
    static const QString TRACKTYPEDATA = "tra"; // 轨迹类型
    static const QString TARLIST = "tra_list";  // 轨迹链表
    static const QString BOWLS = "bowls";       // 碳碗
    static const QString BOWL = "bowl";         // 碳碗
    static const QString CARBONINFO = "carbon_info";
    static const QString NAME = "name";          // 炭块名称
    static const QString WORKNAME = "work_name"; // 碳碗名称
};
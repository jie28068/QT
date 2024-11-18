#ifndef DEFINE_H
#define DEFINE_H
#include <QString>

namespace Define
{
    // HMI消息定义
    static const QString robot_status = "robot_status";                 // 机械臂状态 -2异常 1正常 3在原位 4工作中
    static const QString hardware_status = "hardware_status";           // 硬件状态 3异常 4正常
    static const QString knife_speed = "knife_speed";                   // 刀具转速
    static const QString carbon_status = "carbon_status";               // 碳块就位 3已就位 4未就位
    static const QString air_pressure = "air_pressure";                 // 设备气压
    static const QString negative_pressure = "negative_pressure";       // 收尘负压
    static const QString carbon_moved = "carbon_moved";                 // 碳块已送走 3已送走 4未送走
    static const QString pass_carbon_status = "pass_carbon_mode";       // 放过炭块状态 3放过炭块 4正常清理
    static const QString m011_status = "m011_status";                   // m011故障 3故障 4正常
    static const QString m013_status = "m013_status";                   // m013故障 3故障 4正常
    static const QString m014_status = "m014_status";                   // m014故障 3故障 4正常
    static const QString pneumatic_oil_status = "oil_status";           // 气动油低 3故障 4正常
    static const QString hardware_working_status = "hardware_status";   // 系统运行状态 3运行 4停止
    static const QString carbon_up_status = "carbon_up_status";         // 顶升机构状态 3故障 4正常
    static const QString carbon_down_status = "carbon_down_status";     // 下降机构状态 3故障 4正常
    static const QString carbon_enter_safe_status = "entrance_status";  //  碳块入口安全信号 3不安全 4安全
    static const QString carbon_exit_safe_status = "exit_status";       //  碳块出口安全信号 3不安全 4安全
    static const QString safe_door = "safe_door";                       //  安全门信号 3开门 4关门
    static const QString plc_system_control = "plc_system_control";     // 系统状态 "plc_system_start": 3, "plc_system_stop": 4
    static const QString pass_carbon_mode = "pass_carbon_mode";         // 放过炭块模式 "pass_carbon": 3, "stop_pass_carbon": 4
    static const QString robot_working_status = "robot_working_status"; // 机械臂工作状态 "robot_working": 3, "robot_free": 4
    // 状态
    static const QString STATUS_UNKNOWN = "未知";
    static const QString STATUS_ONLINE = "正常";
    static const QString STATUS_OFFLINE = "故障";
    static const QString STATUS_ERROR = "异常";
    static const QString STATUS_WARNING = "警告";

    static const QString STATUS_SITU = "在原位";
    static const QString STATUS_WORK = "工作中";
    static const QString STATUS_ONPLACE = "已就位";
    static const QString STATUS_OFFPLACE = "未就位";
    static const QString STATUS_ONAWAY = "已送走";
    static const QString STATUS_OFFAWAY = "未送走";
    static const QString STATUS_UPTAR = "放过炭块";
    static const QString STATUS_CLEAR = "正常清理";
    static const QString STATUS_RUN = "运行";
    static const QString STATUS_STOP = "停止";
    static const QString STATUS_SAFEY = "安全";
    static const QString STATUS_NOSAFEY = "不安全";
    static const QString STATUS_OPENDOOR = "开门";
    static const QString STATUS_OFFDOOR = "关门";
    // end
    //  页面切换按钮图片大小
    static const int PAGE_ICON_SIZE = 40;
    // 速度图标大小
    static const int SPEED_ICON_SIZE = 50;

    //  树样式
    static const QString PAGE_TREE_QSS = R"(
        QTreeView{
        border:1px solid #C0DCF2;
        selection-background-color: #F9D699;
        selection-color:#386487;
        alternate-background-color:#DAEFFF;
        gridline-color:#C0DCF2;
        font-size:20px;
        }

        QTreeView::branch:closed:has-children{
        margin:3px;
        border-image:url(":/images/resources/ping.png");
        }

        QTreeView::branch:open:has-children{
        margin:2px;
        border-image:url(":/images/resources/xia.png");
        }

        QTreeView,QSplitter::handle,QTreeView::branch{
        background:#EAF7FF;
        }

        QTreeView::item:selected{
        color:#386487;
        background:qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #DEF0FE,stop:1 #C0DEF6);
        }

        QTreeView::item:hover,QHeaderView{
        color:#386487;
        background:qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #F2F9FF,stop:1 #DAEFFF);
        }

        QTreeView::item{
        padding:1px;
        margin:0px;
        }
        QLineEdit,QComboBox{
            background-color: #FFFFFF;
            border: 1px solid #ccc;
            color: #333;
        }

        QDoubleSpinBox,QSpinBox {
            background-color: #FFFFFF;
            border: 1px solid #ccc;
            color: #333;
        }
            )";
    // 表格样式
    static const QString PAGE_TABLE_QSS = R"(QTableWidget, QTableView {
            color: rgb(255, 255, 255);
            outline: none;
            border: none;
            background-color: #1B2E4A; /* 表格背景色 */
            selection-color: #036eb7; /* 选中文字颜色 */
            selection-background-color: #d1e0ef; /* 选中背景色 */
            }
            QTableWidget::item{
                border: 1px solid #3386F4;
            }

            QHeaderView::section {
                background-color: #20C1FF;
            }
            QHeaderView::section:horizontal {
                color: rgb(255, 255, 255);
                font-weight: bold;
                font-size:18px;
                border-top: 1px solid #3386F4;
                border-left: none;
                border-right: 1px solid #3386F4;
                border-bottom: 1px solid #3386F4;
            }
            QHeaderView::section:vertical {
                border-top: none;
                border-left: 1px solid #3386F4;
                border-right: 1px solid #3386F4;
                border-bottom: 1px solid #3386F4;
            })";

}

#endif
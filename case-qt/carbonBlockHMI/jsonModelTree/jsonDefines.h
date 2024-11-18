#pragma once

#include <QString>

namespace JsonDefines
{
    enum HMI
    {
        UPLOAD_TRAJ = 1,     // 上传已生成轨迹
        UPLOAD_TEMPLATE = 2, // 上传轨迹生成模板
        CREATE_TRAJ = 3,     // 生成轨迹
        UPLOAD_PARAMS = 4,   // 上传参数
        UPDATA_PARAMS = 5,   // 更新参数
        START_WORK = 6,      // 运行算法
    };

    static const QString TRACKTYPE = "type";    // 轨迹类型
    static const QString TRACKTYPEDATA = "tra"; // 轨迹类型
    static const QString TARLIST = "tra_list";  // 轨迹链表
    static const QString BOWLS = "bowls";       // 碳碗
    static const QString BOWL = "bowl";         // 碳碗
    static const QString CARBONINFO = "carbon_info";
    static const QString NAME = "name";          // 炭块名称
    static const QString WORKNAME = "work_name"; // 碳碗名称

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
        border-image:url("/home/ray/myGitproject/my-case/QT/carbonBlockHMI/ping.png");
        }

        QTreeView::branch:open:has-children{
        margin:2px;
        border-image:url("/home/ray/myGitproject/my-case/QT/carbonBlockHMI/xia.png");
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
};
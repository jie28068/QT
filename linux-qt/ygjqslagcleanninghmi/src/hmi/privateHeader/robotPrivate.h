#pragma once

#include <QCamera>     // 摄像头
#include <QCameraInfo> // 摄像头信息
#include <QCameraViewfinder>
#include <QCameraImageCapture> // 拍照
#include <QVideoProbe>         // 视频
#include <QVideoWidget>        // 视频显示窗口
#include <QMediaPlayer>
#include <vector>
#include <QNetworkRequest>
#include <QColorDialog>
#include <QSignalMapper>
#include <QPixmap>
#include <QEvent>
#include <QDir>
#include <QStyle>
#include <QSound>

#include "qtnode.h"

class RobotPrivate
{
public:
    RobotPrivate();
    ~RobotPrivate();

    QtNode *qt_node_;
    std::vector<QString> is_camera_url;   // 视频流url
    std::vector<bool> get_camera_info;    // 是否获取到camera信息
    std::vector<bool> cur_camera_channel; // 当前camera通道
    QRect widget_rect_;
    QMediaPlayer _player1;
    QMediaPlayer _player2;
    QVideoWidget *_vw1;
    QVideoWidget *_vw2;
    QString log_content; // 操作日志内容
    bool is_warning_dialog;
    bool is_fullscreen; // 是否全屏
    /// @brief 按钮信号映射器
    QSignalMapper *mapper;
    /// @brief 上下位机是否连接
    bool is_connect;
    hmi_msgs::HmiStatus current_msg;
    QSound *sound;
    bool sound_flag = false; // 是否播放背景音乐
};

RobotPrivate::RobotPrivate()
{
    qt_node_ = nullptr;
    is_camera_url = std::vector<QString>();
    get_camera_info = std::vector<bool>();
    cur_camera_channel = std::vector<bool>();
    widget_rect_ = QRect();
    _vw1 = nullptr;
    _vw2 = nullptr;
    log_content = QString();
    is_warning_dialog = false;
    is_fullscreen = false;
    mapper = nullptr;
    is_connect = false;
    current_msg = hmi_msgs::HmiStatus();
    sound = new QSound(":/resource/sound/alarm.wav");
}

RobotPrivate::~RobotPrivate() {}
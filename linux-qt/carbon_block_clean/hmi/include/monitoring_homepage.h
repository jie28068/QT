#ifndef MONITORING_HOMEPAGE_H
#define MONITORING_HOMEPAGE_H

#include <QWidget>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <QTableWidget>
#include <QHeaderView>
// #include <opencv2/opencv.hpp>

#include "hmi_datatypes.h"
#include "glog/logging.h"

#include <QButtonGroup>
#include <QPushButton>

class DialPlateWidget;
class HostComputeClient;
namespace Ui
{
    class Monitoring_homepage;
}

class Monitoring_homepage : public QWidget
{
    Q_OBJECT

public:
    explicit Monitoring_homepage(QSharedPointer<HostComputeClient> clint, QWidget *parent = nullptr);
    ~Monitoring_homepage();
    /// @brief  刷新数据
    /// @param str
    void updateMessage(QJsonDocument jsonDoc);
    /// @brief  更新连接状态
    /// @param status
    void updateConnectionStatus(bool status);

private slots:
    /// @brief  刀具检修
    void onToolMaintenance();
    /// @brief  返回原位
    void onReturnOriginalPosition();
    /// @brief  高速
    void onHighSpeed();
    /// @brief  低速
    void onLowSpeed();

private:
    Ui::Monitoring_homepage *ui;
    void init_image();
    /// @brief 初始化表格
    void init_tableWidget();
    /// @brief 初始化系统信息
    void init_system_label();
    /// @brief 初始化仪表盘
    void init_dial();
    /// @brief 弹窗
    /// @param str
    /// @param type 0:提示 1:警告
    /// @return
    bool messageDialog(const QString &str, int type = 0);

private:
    void speedButton();
    void ControlButton();

private:
    QButtonGroup *m_speedgroupButton; // 速度按钮组
    int m_speedgroupButtonID = 0;     // 速度切换按钮组id

    QButtonGroup *m_controlgroupButton; // 控制按钮组
    int m_controlgroupButtonID = 0;     // 控制切换按钮组id

    bool m_connectionStatus = false; // 连接状态
    QSharedPointer<HostComputeClient> m_clint;
};

#endif // MONITORING_HOMEPAGE_H

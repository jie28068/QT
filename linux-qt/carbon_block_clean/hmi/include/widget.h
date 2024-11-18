#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QDateTime>
#include <QTimer>
#include <QDebug>
#include <QButtonGroup>

#include "monitoring_homepage.h"
#include "data_check.h"
#include "report_check.h"
#include "log_management.h"
#include "minterface.h"

#include <QPushButton>
#include <QDateTime>

class DataQueryPage;
class TarPage;
class HostComputeClient;
class AdjustParamPage;
class ServerThread;
class SchedulingServiceImpl;
QT_BEGIN_NAMESPACE
namespace Ui
{
    class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget, public IObserver
{
    Q_OBJECT

public:
    Widget(int argc, char **argv, QWidget *parent = nullptr);
    ~Widget();

    ///@brief 按钮组
    void PageButton();
    void ControlButton();
    /// @brief 初始化时间即心跳检测
    void initTime();
    /// @brief 心跳获取信息
    /// @param str 数据
    void onServerTimeMessage(const QString &str) override;

private:
    Ui::Widget *ui;
    Qt::WindowStates m_OldWindowState = Qt::WindowNoState;
    Qt::WindowStates m_WindowState = Qt::WindowNoState;
    Monitoring_homepage *ui_page1;
    Data_check *ui_page2;
    Report_check *ui_page3;
    DataQueryPage *ui_page4;
    AdjustParamPage *ui_page5;
    TarPage *ui_page6;
    /// @brief 初始化服务器与客户端
    void initServerClint();
    void initUI();
    /// @brief 设置操作按钮样式
    void setControlButtonQSS();
    /// @brief 弹窗
    /// @param str
    /// @param type 0:提示 1:警告
    /// @return
    bool messageDialog(const QString &str, int type = 0);

private:
    QSharedPointer<ServerThread> m_thread;            // 服务端线程
    QSharedPointer<SchedulingServiceImpl> m_services; // 服务端
    QSharedPointer<HostComputeClient> m_client;       // 客服端
    QButtonGroup *m_groupButton;                      // 页面按钮组
    int m_groupButtonID = 0;                          // 页面切换按钮组id
    /// @brief 跟踪最后一次调用时间
    QDateTime lastCallbackTime;

    bool m_robotStatus = false;  // 机器臂运行
    bool m_systemStatus = false; // PLC系统运行
    bool m_carbonStatus = false; // 正常清理

private slots:
    void changeEvent(QEvent *event);
    void update_system_time_slot();
    void on_pushButton_minimize_clicked();
    void on_pushButton_close_clicked();
    void checkPlcStatusDataCallback();
    void onCurrentPageChanged(int index);
    /// @brief 机器臂停止
    void onRobotArmStop();
    /// @brief 系统停止
    void onSystemStop();
    /// @brief 正常清理
    void onNormalClean();
};
#endif // WIDGET_H

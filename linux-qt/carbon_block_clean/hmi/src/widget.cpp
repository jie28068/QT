#include "widget.h"
#include "ui_widget.h"
#include "define.h"
#include <QPixmap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "customButton.h"
#include "dataquerypage.h"
#include "tarpage.h"
#include "adjustparampage.h"
#include "cuelightlabel.h"
#include "jsonDefines.h"
#include "customDialog.h"

#include "grpcClinet.h"
#include "grpcDataThead.h"
#include "grpcServer.h"

Widget::Widget(int argc, char **argv, QWidget *parent)
    : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
    initServerClint();
    PageButton();
    ControlButton();
    initUI();
    initTime();
    setControlButtonQSS();
}
Widget::~Widget()
{
    // 请求线程停止
    m_thread->quit(); // 请求线程退出事件循环
    m_thread->wait(); // 等待线程真正结束
    delete ui;
}

void Widget::initServerClint()
{
    m_client = QSharedPointer<HostComputeClient>(new HostComputeClient(grpc::CreateChannel("192.168.20.49:50052", grpc::InsecureChannelCredentials())));
    m_services = QSharedPointer<SchedulingServiceImpl>(new SchedulingServiceImpl());
    m_services->registerObserver(this);
    m_thread = QSharedPointer<ServerThread>(new ServerThread(m_services, this));
    m_thread->start();
}

void Widget::initUI()
{
    // 窗口定义
    ui_page1 = new Monitoring_homepage(m_client);
    ui_page2 = new Data_check();
    ui_page3 = new Report_check();
    ui_page4 = new DataQueryPage(this);
    ui_page5 = new AdjustParamPage(m_client, this);

    ui_page6 = new TarPage(m_client, this);

    ui->stackedWidget->addWidget(ui_page1);
    ui->stackedWidget->addWidget(ui_page2);
    ui->stackedWidget->addWidget(ui_page3);
    ui->stackedWidget->addWidget(ui_page4);
    ui->stackedWidget->addWidget(ui_page5);
    ui->stackedWidget->addWidget(ui_page6);

    // 默认为监控主页页面
    ui->pushButton->setHighlighted(true);
    ui->stackedWidget->setCurrentIndex(0);

    /// 初始化通信灯
    ui->label_communication->setStatus(CueLightLabel::Offline);
    ui->label_communication->setBreathingEffectEnabled(true);
    ui->label_communication->setToolTip("通信失败");

    connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, &Widget::onCurrentPageChanged);
}

void Widget::setControlButtonQSS()
{
    ui->pushButton_7->setPushButtonStyle(false);
    ui->pushButton_8->setPushButtonStyle(false);
    ui->pushButton_9->setPushButtonStyle(false);
    ui->pushButton_10->setPushButtonStyle(false);
    m_robotStatus = false;
    m_systemStatus = false;
    m_carbonStatus = false;
    ui->pushButton_8->setTextCo("机器臂停止");
    ui->pushButton_9->setTextCo("放过炭块");
    ui->pushButton_10->setTextCo("系统停止");
}

bool Widget::messageDialog(const QString &str, int type)
{
    CustomDialog Dialog(str, type, this);
    if (Dialog.exec() == QDialog::Accepted)
    {
        return true;
    }
    return false;
}

void Widget::PageButton()
{
    QPixmap icon;
    QPixmap iconHightlighted;
    icon = (QPixmap(":/images/resources/jiankongzhuye.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/jiankongzhuye@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton->setTextIcon("监控主页", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/shujuchaxun.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/shujuchaxun@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_2->setTextIcon("数据查询", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/baobiaochaxun.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/baobiaochaxun@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_3->setTextIcon("报表查询", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/rizhiguanli.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/rizhiguanli@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_4->setTextIcon("日志管理", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/canshutiaojie.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/canshutiaojie@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_5->setTextIcon("参数调节", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/canshutiaojie.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/canshutiaojie@select.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_6->setTextIcon("炭块轨迹", icon, iconHightlighted);

    m_groupButton = new QButtonGroup(this);
    m_groupButton->setExclusive(true);
    m_groupButton->addButton(ui->pushButton, m_groupButtonID++);
    m_groupButton->addButton(ui->pushButton_2, m_groupButtonID++);
    m_groupButton->addButton(ui->pushButton_3, m_groupButtonID++);
    m_groupButton->addButton(ui->pushButton_4, m_groupButtonID++);
    m_groupButton->addButton(ui->pushButton_5, m_groupButtonID++);
    m_groupButton->addButton(ui->pushButton_6, m_groupButtonID++);

    connect(ui->pushButton, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(0); });
    connect(ui->pushButton_2, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(1); });
    connect(ui->pushButton_3, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(2); });
    connect(ui->pushButton_4, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(3); });
    connect(ui->pushButton_5, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(4); });
    connect(ui->pushButton_6, &QPushButton::clicked, [=]()
            { ui->stackedWidget->setCurrentIndex(5); });
    connect(m_groupButton, QOverload<int>::of(&QButtonGroup::buttonClicked), [=](int id)
            {
                    for (int i = 0; i < m_groupButton->buttons().size(); ++i)
                    {
                        auto button = m_groupButton->button(i);
                        CustomButton *mbutton = qobject_cast<CustomButton *>(button);
                        if (mbutton)
                        {
                            mbutton->setHighlighted(i == id);
                        }
                    } });
}

void Widget::ControlButton()
{
    QPixmap icon;
    icon = (QPixmap(":/images/resources/stop.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_7->setTextIcon("服务重启", icon);

    icon = (QPixmap(":/images/resources/stop.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_8->setTextIcon("机械臂停止", icon);

    icon = (QPixmap(":/images/resources/stop.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_9->setTextIcon("放过炭块", icon);

    icon = (QPixmap(":/images/resources/qidong.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_10->setTextIcon("系统停止", icon);

    connect(ui->pushButton_8, &QPushButton::clicked, this, &Widget::onRobotArmStop);
    connect(ui->pushButton_9, &QPushButton::clicked, this, &Widget::onNormalClean);
    connect(ui->pushButton_10, &QPushButton::clicked, this, &Widget::onSystemStop);
}

void Widget::initTime()
{
    // 时间刷新
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Widget::update_system_time_slot);
    timer->start();
    // 心跳检测
    QTimer *timer2 = new QTimer(this);
    connect(timer2, &QTimer::timeout, this, &Widget::checkPlcStatusDataCallback);
    timer2->start(2000);
    // 初始化最后一次调用时间
    lastCallbackTime = QDateTime::currentDateTime();
}

void Widget::onServerTimeMessage(const QString &str)
{
    // 更新最后一次调用时间
    lastCallbackTime = QDateTime::currentDateTime();
    if (ui_page1)
    {
        QJsonDocument jsonDoc = QJsonDocument::fromJson(str.toUtf8());
        if (jsonDoc.isNull())
        {
            LOG(ERROR) << "json解析失败！";
            return;
        }
        // todo
        ui_page1->updateMessage(jsonDoc);
        ///
        if (jsonDoc.isObject())
        {
            QJsonObject obj = jsonDoc.object();
            // 机器臂运行 机器臂停止
            int robot_working_status = obj[Define::robot_working_status].toInt();
            if (robot_working_status == 3)
            {
                ui->pushButton_8->setPushButtonStyle(true);
                m_robotStatus = true; // 机器臂运行
                ui->pushButton_8->setTextCo("机器臂运行");
            }
            else if (robot_working_status == 4)
            {
                ui->pushButton_8->setPushButtonStyle(false);
                m_robotStatus = false;
                ui->pushButton_8->setTextCo("机器臂停止");
            }
            // 正常清理 放过炭块
            int pass_carbon_mode = obj[Define::pass_carbon_mode].toInt();
            if (pass_carbon_mode == 3)
            {
                ui->pushButton_9->setPushButtonStyle(true);
                m_carbonStatus = true;
                ui->pushButton_9->setTextCo("正常清理");
            }
            else if (pass_carbon_mode == 4)
            {
                ui->pushButton_9->setPushButtonStyle(false);
                m_carbonStatus = false;
                ui->pushButton_9->setTextCo("放过炭块");
            }
            // PLC系统运行 PLC系统停止
            int plc_system_control = obj[Define::plc_system_control].toInt();
            if (plc_system_control == 3)
            {
                ui->pushButton_10->setPushButtonStyle(true);
                m_systemStatus = true;
                ui->pushButton_10->setTextCo("系统运行");
            }
            else if (plc_system_control == 4)
            {
                ui->pushButton_10->setPushButtonStyle(false);
                m_systemStatus = false;
                ui->pushButton_10->setTextCo("系统停止");
            }
        }
    }
}

void Widget::update_system_time_slot()
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui->label_time->setText(dateTime.toString("yyyy-MM-dd hh:mm:ss"));
}

void Widget::changeEvent(QEvent *event)
{
    if (event->type() == QEvent::WindowStateChange)
    {
        // qDebug() << "curState: " << this->windowState() << " | oldState: " << stateEvent->oldState();
        if (m_WindowState == Qt::WindowMinimized && this->windowState() != Qt::WindowFullScreen)
        {
            if (m_OldWindowState == Qt::WindowFullScreen)
            {
                this->setWindowState(Qt::WindowFullScreen);
            }
        }
        m_WindowState = this->windowState();
    }
    QWidget::changeEvent(event);
}

void Widget::on_pushButton_minimize_clicked()
{
    m_OldWindowState = this->windowState();
    this->setWindowState(Qt::WindowMinimized);
}

void Widget::on_pushButton_close_clicked()
{
    this->close();
}

void Widget::checkPlcStatusDataCallback()
{
    if (ui_page1)
    {
        QDateTime currentTime = QDateTime::currentDateTime();
        int elapsed = lastCallbackTime.msecsTo(currentTime);
        // 检查是否在过去的一段时间内回调函数被调用
        bool g = (elapsed > 2000 ? false : true);
        if (!g)
        {
            ui->label_communication->setStatus(CueLightLabel::Offline);
            ui->label_communication->setBreathingEffectEnabled(true);
            ui->label_communication->setToolTip("通信失败");
            ui_page1->updateConnectionStatus(false);
            setControlButtonQSS();
        }
        else
        {
            ui->label_communication->setStatus(CueLightLabel::Online);
            ui->label_communication->setBreathingEffectEnabled(false);
            ui->label_communication->setToolTip("通信成功");
            ui_page1->updateConnectionStatus(true);
        }
    }
}

void Widget::onCurrentPageChanged(int index)
{
    if (index == 5)
    {
        ui_page6->carbonList();
    }
    else if (index == 4)
    {
        ui_page5->updateJsonData();
    }
}

void Widget::onRobotArmStop()
{
    if (!messageDialog("机器臂停止"))
        return;
    // 机器臂停止
    int32_t task = JsonDefines::ASK_FOR_CARBON;
    std::string str = !m_robotStatus ? "robot_working" : "robot_free";
    m_client->ScheduleTask(task, str);
}

void Widget::onSystemStop()
{
    if (!messageDialog("系统停止"))
        return;
    // PLC系统停止
    int32_t task = JsonDefines::PLC_STATUS;
    std::string str = !m_systemStatus ? "plc_system_start" : "plc_system_stop";
    m_client->ScheduleTask(task, str);
}

void Widget::onNormalClean()
{
    if (!messageDialog("放过炭块"))
        return;
    // 放过炭块
    int32_t task = JsonDefines::LINE_STATUS;
    std::string str = !m_carbonStatus ? "pass_carbon" : "stop_pass_carbon";
    m_client->ScheduleTask(task, str);
}

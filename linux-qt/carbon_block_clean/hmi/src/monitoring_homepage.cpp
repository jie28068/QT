#include "monitoring_homepage.h"
#include "ui_monitoring_homepage.h"
#include "define.h"
#include "jsonDefines.h"
#include "customButton.h"
#include "dialPlateWidget.h"
#include "grpcClinet.h"
#include "customDialog.h"

#include <QPixmap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
Monitoring_homepage::Monitoring_homepage(QSharedPointer<HostComputeClient> clint, QWidget *parent) : QWidget(parent),
                                                                                                     ui(new Ui::Monitoring_homepage), m_clint(clint)
{
    ui->setupUi(this);
    init_image();
    init_tableWidget();
    speedButton();
    ControlButton();
    init_system_label();
    init_dial();
}

Monitoring_homepage::~Monitoring_homepage()
{
    delete ui;
}

void Monitoring_homepage::init_image()
{
    // // 加载图片
    // cv::Mat img = cv::imread("/home/pc/Desktop/images.jpg");
    // // 检查图片是否加载成功
    // if (img.empty())
    // {
    //     qDebug() << "图片加载失败";
    //     return;
    // }
    // qDebug() << "图片加载成功";
    // cv::Mat new_img;
    // cv::cvtColor(img, new_img, cv::COLOR_BGR2RGB);
    // // 将cv::Mat转换为QImage
    // QImage qImage = QImage((const unsigned char *)(new_img.data),
    //                        new_img.cols,
    //                        new_img.rows,
    //                        new_img.step,
    //                        QImage::Format_RGB888);

    // // 将QImage转换为QPixmap
    // QPixmap pixmap = QPixmap::fromImage(qImage);
    // // 图片自适应控件大小
    // ui->label_camera->setScaledContents(true);
    // // 创建一个标签来显示图片
    // ui->label_camera->setPixmap(pixmap);
    // ui->label_camera->show();
}

void Monitoring_homepage::init_tableWidget()
{
    // 设置列数
    ui->tableWidget->setColumnCount(2);
    // 设置行数
    ui->tableWidget->setRowCount(3);
    // 表头标题用QStringList表示
    QStringList headerText;
    headerText << "时间"
               << "内容";
    ui->tableWidget->setHorizontalHeaderLabels(headerText);
    // 设置列宽
    QHeaderView *header = ui->tableWidget->horizontalHeader();
    header->setSectionResizeMode(QHeaderView::ResizeToContents);
    header->setSectionResizeMode(0, QHeaderView::Interactive);
    ui->tableWidget->setColumnWidth(0, 300);
    header->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableWidget->setStyleSheet(Define::PAGE_TABLE_QSS);
}

bool Monitoring_homepage::messageDialog(const QString &str, int type)
{
    CustomDialog Dialog(str, type, this);
    if (Dialog.exec() == QDialog::Accepted)
    {
        return true;
    }
    return false;
}

void Monitoring_homepage::speedButton()
{
    QPixmap icon;
    QPixmap iconHightlighted;
    icon = (QPixmap(":/images/resources/high-speed.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/high-speed-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton->setTextIcon("高速", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/middle-speed.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/middle-speed-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_2->setTextIcon("中速", icon, iconHightlighted);

    // icon = (QPixmap(":/images/resources/low-speed.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    // iconHightlighted = (QPixmap(":/images/resources/low-speed-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    // ui->pushButton_3->setTextIcon("低速", icon, iconHightlighted);

    // icon = (QPixmap(":/images/resources/super-low-speed.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    // iconHightlighted = (QPixmap(":/images/resources/super-low-speed-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    // ui->pushButton_4->setTextIcon("超低速", icon, iconHightlighted);

    m_speedgroupButton = new QButtonGroup(this);
    m_speedgroupButton->setExclusive(true);
    m_speedgroupButton->addButton(ui->pushButton, m_speedgroupButtonID++);
    m_speedgroupButton->addButton(ui->pushButton_2, m_speedgroupButtonID++);
    // m_speedgroupButton->addButton(ui->pushButton_3, m_speedgroupButtonID++);
    // m_speedgroupButton->addButton(ui->pushButton_4, m_speedgroupButtonID++);

    connect(m_speedgroupButton, QOverload<int>::of(&QButtonGroup::buttonClicked), [=](int id)
            {
                    for (int i = 0; i < m_speedgroupButton->buttons().size(); ++i)
                    {
                        auto button = m_speedgroupButton->button(i);
                        CustomButton *mbutton = qobject_cast<CustomButton *>(button);
                        if (mbutton)
                        {
                            mbutton->setHighlighted(i == id);
                        }
                    } });
    connect(ui->pushButton, &QPushButton::clicked, this, &Monitoring_homepage::onHighSpeed);
    connect(ui->pushButton_2, &QPushButton::clicked, this, &Monitoring_homepage::onLowSpeed);
}

void Monitoring_homepage::ControlButton()
{
    QPixmap icon;
    QPixmap iconHightlighted;
    icon = (QPixmap(":/images/resources/daojujianxiu.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/daojujianxiu-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_5->setTextIcon("刀具检修", icon, iconHightlighted);

    icon = (QPixmap(":/images/resources/fanhuiyuanwei.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    iconHightlighted = (QPixmap(":/images/resources/fanhuiyuanwei-s.svg").scaled(QSize(Define::SPEED_ICON_SIZE, Define::SPEED_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->pushButton_7->setTextIcon("返回原位", icon, iconHightlighted);

    m_controlgroupButton = new QButtonGroup(this);
    m_controlgroupButton->setExclusive(true);
    m_controlgroupButton->addButton(ui->pushButton_5, m_controlgroupButtonID++);
    m_controlgroupButton->addButton(ui->pushButton_7, m_controlgroupButtonID++);

    connect(m_controlgroupButton, QOverload<int>::of(&QButtonGroup::buttonClicked), [=](int id)
            {
                    for (int i = 0; i < m_controlgroupButton->buttons().size(); ++i)
                    {
                        auto button = m_controlgroupButton->button(i);
                        CustomButton *mbutton = qobject_cast<CustomButton *>(button);
                        if (mbutton)
                        {
                            mbutton->setHighlighted(i == id);
                        }
                    } });
    connect(ui->pushButton_5, &QPushButton::clicked, this, &Monitoring_homepage::onToolMaintenance);
    connect(ui->pushButton_7, &QPushButton::clicked, this, &Monitoring_homepage::onReturnOriginalPosition);
}

void Monitoring_homepage::onToolMaintenance()
{
    if (!messageDialog("刀具检修"))
        return;
    int32_t task = JsonDefines::CHECK_KNIFE;
    auto reply = m_clint->ScheduleTask(task);
}

void Monitoring_homepage::onReturnOriginalPosition()
{
    if (!messageDialog("返回原位"))
        return;
    int32_t task = JsonDefines::BACK_HOME;
    auto reply = m_clint->ScheduleTask(task);
}

void Monitoring_homepage::onHighSpeed()
{
    int32_t task = JsonDefines::UPLOAD_TRAJ;
    // auto reply = m_clint->ScheduleTask(task);
}

void Monitoring_homepage::onLowSpeed()
{
    int32_t task = JsonDefines::UPLOAD_TRAJ;
    // auto reply = m_clint->ScheduleTask(task);
}

void Monitoring_homepage::init_system_label()
{
    // 机械臂状态
    ui->widget_robot_status->setLabelIcon(QString("background: url(:/images/resources/svg/机器人位置-icon.svg)"));
    ui->widget_robot_status->setLabelTitle("机械臂状态");
    ui->widget_robot_status->setLabelText(Define::STATUS_UNKNOWN);

    // 硬件状态
    ui->widget_hardware_status->setLabelIcon(QString("background: url(:/images/resources/svg/钳制器状态-icon.svg)"));
    ui->widget_hardware_status->setLabelTitle("硬件状态");
    ui->widget_hardware_status->setLabelText(Define::STATUS_UNKNOWN);

    // 碳块就位
    ui->widget_carbon_status->setLabelIcon(QString("background: url(:/images/resources/svg/风镐-icon.svg)"));
    ui->widget_carbon_status->setLabelTitle("碳块就位");
    ui->widget_carbon_status->setLabelText(Define::STATUS_UNKNOWN);

    // 碳块送走
    ui->widget_carbon_moved->setLabelIcon(QString("background: url(:/images/resources/abb.svg)"));
    ui->widget_carbon_moved->setLabelTitle("碳块送走");
    ui->widget_carbon_moved->setLabelText(Define::STATUS_UNKNOWN);

    // 放过炭块状态
    ui->widget_pass_carbon_status->setLabelIcon(QString("background: url(:/images/resources/clean.svg)"));
    ui->widget_pass_carbon_status->setLabelTitle("放过炭块状态");
    ui->widget_pass_carbon_status->setLabelText(Define::STATUS_UNKNOWN);

    // m011故障
    ui->widget_m011_status->setLabelIcon(QString("background: url(:/images/resources/svg/防尘房急停按钮-icon.svg)"));
    ui->widget_m011_status->setLabelTitle("m011故障");
    ui->widget_m011_status->setLabelText(Define::STATUS_UNKNOWN);
    // m013故障
    ui->widget_m013_status->setLabelIcon(QString("background: url(:/images/resources/svg/防尘房急停按钮-icon.svg)"));
    ui->widget_m013_status->setLabelTitle("m013故障");
    ui->widget_m013_status->setLabelText(Define::STATUS_UNKNOWN);
    // m014故障
    ui->widget_m014_status->setLabelIcon(QString("background: url(:/images/resources/svg/防尘房急停按钮-icon.svg)"));
    ui->widget_m014_status->setLabelTitle("m014故障");
    ui->widget_m014_status->setLabelText(Define::STATUS_UNKNOWN);

    // 气动油低
    ui->widget_pneumatic_oil_status->setLabelIcon(QString("background: url(:/images/resources/svg/控制柜温度-icon.svg)"));
    ui->widget_pneumatic_oil_status->setLabelTitle("气动油低");
    ui->widget_pneumatic_oil_status->setLabelText(Define::STATUS_UNKNOWN);

    // 系统运行状态
    ui->widget_hardware_working_status->setLabelIcon(QString("background: url(:/images/resources/svg/雷达状态-icon.svg)"));
    ui->widget_hardware_working_status->setLabelTitle("系统运行状态");
    ui->widget_hardware_working_status->setLabelText(Define::STATUS_UNKNOWN);

    // 顶升机构状态
    ui->widget_carbon_up_status->setLabelIcon(QString("background: url(:/images/resources/svg/操作模式-icon.svg)"));
    ui->widget_carbon_up_status->setLabelTitle("顶升机构状态");
    ui->widget_carbon_up_status->setLabelText(Define::STATUS_UNKNOWN);

    // 下降机构状态
    ui->widget_carbon_down_status->setLabelIcon(QString("background: url(:/images/resources/svg/操作模式-icon.svg)"));
    ui->widget_carbon_down_status->setLabelTitle("下降机构状态");
    ui->widget_carbon_down_status->setLabelText(Define::STATUS_UNKNOWN);

    // 碳块出口信号
    ui->widget_carbon_exit_safe_status->setLabelIcon(QString("background: url(:/images/resources/svg/楼梯口安全门-icon.svg)"));
    ui->widget_carbon_exit_safe_status->setLabelTitle("碳块出口信号");
    ui->widget_carbon_exit_safe_status->setLabelText(Define::STATUS_UNKNOWN);

    // 碳块入口信号
    ui->widget_carbon_enter_safe_status->setLabelIcon(QString("background: url(:/images/resources/svg/楼梯口安全门-icon.svg)"));
    ui->widget_carbon_enter_safe_status->setLabelTitle("碳块入口信号");
    ui->widget_carbon_enter_safe_status->setLabelText(Define::STATUS_UNKNOWN);

    // 安全门信号
    ui->widget_safe_door->setLabelIcon(QString("background: url(:/images/resources/svg/电气室安全门-icon.svg)"));
    ui->widget_safe_door->setLabelTitle("安全门信号");
    ui->widget_safe_door->setLabelText(Define::STATUS_UNKNOWN);
}

void Monitoring_homepage::init_dial()
{
    ui->speedDashboard->setContent("转速");
    ui->dustPressure->setContent("气压", "Mpa");
    ui->equipmentPressure->setContent("负压", "Pa");

    ui->speedDashboard->setMinValue(0);
    ui->speedDashboard->setMaxValue(120);
    ui->dustPressure->setMinValue(0);
    ui->dustPressure->setMaxValue(1);
    ui->equipmentPressure->setMinValue(-3000);
    ui->equipmentPressure->setMaxValue(0);
}

void Monitoring_homepage::updateMessage(QJsonDocument jsonDoc)
{
    QByteArray jsonData = jsonDoc.toJson();
    QJsonDocument doc = QJsonDocument::fromJson(jsonData);
    if (doc.isObject())
    {
        updateConnectionStatus(true);
        QJsonObject obj = doc.object();
        // 刀具转速
        int knife_speed = obj[Define::knife_speed].toInt();
        ui->speedDashboard->setValue(knife_speed);
        ui->label_device_pressure_value_5->setText(QString("设备转速：%1").arg(QString::number(knife_speed)));
        // 设备气压
        int air_pressure = obj[Define::air_pressure].toInt();
        ui->dustPressure->setValue(air_pressure);
        ui->label_device_pressure_value_4->setText(QString("设备气压：%1Mpa").arg(QString::number(air_pressure)));
        // 收尘气压
        int negative_pressure = obj[Define::negative_pressure].toInt();
        ui->equipmentPressure->setValue(negative_pressure);
        ui->label_device_pressure_value_3->setText(QString("收尘负压：%1Pa").arg(QString::number(negative_pressure)));
        // 机械臂状态
        int robot_status = obj[Define::robot_status].toInt();
        if (robot_status == 1)
        {
            ui->widget_robot_status->setLabelText(Define::STATUS_ONLINE);
        }
        else if (robot_status == 2)
        {
            ui->widget_robot_status->setLabelText(Define::STATUS_ERROR);
        }
        else if (robot_status == 3)
        {
            ui->widget_robot_status->setLabelText(Define::STATUS_SITU);
        }
        else if (robot_status == 4)
        {
            ui->widget_robot_status->setLabelText(Define::STATUS_WORK);
        }
        // 硬件状态
        int hardware_status = obj[Define::hardware_status].toInt();
        if (hardware_status == 3)
        {
            ui->widget_hardware_status->setLabelText(Define::STATUS_ERROR);
        }
        else if (hardware_status == 4)
        {
            ui->widget_hardware_status->setLabelText(Define::STATUS_ONLINE);
        }
        // 碳块就位
        int carbon_status = obj[Define::carbon_status].toInt();
        if (carbon_status == 3)
        {
            ui->widget_carbon_status->setLabelText(Define::STATUS_ONPLACE);
        }
        else if (carbon_status == 4)
        {
            ui->widget_carbon_status->setLabelText(Define::STATUS_OFFPLACE);
        }
        // 碳块送走
        int carbon_moved = obj[Define::carbon_moved].toInt();
        if (carbon_moved == 3)
        {
            ui->widget_carbon_moved->setLabelText(Define::STATUS_ONAWAY);
        }
        else if (carbon_moved == 4)
        {
            ui->widget_carbon_moved->setLabelText(Define::STATUS_OFFAWAY);
        }
        // 放过炭块状态
        int pass_carbon_status = obj[Define::pass_carbon_status].toInt();
        if (pass_carbon_status == 3)
        {
            ui->widget_pass_carbon_status->setLabelText(Define::STATUS_UPTAR);
        }
        else if (pass_carbon_status == 4)
        {
            ui->widget_pass_carbon_status->setLabelText(Define::STATUS_CLEAR);
        }
        // m011故障
        int m011_status = obj[Define::m011_status].toInt();
        if (m011_status == 3)
        {
            ui->widget_m011_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (m011_status == 4)
        {
            ui->widget_m011_status->setLabelText(Define::STATUS_ONLINE);
        }
        // m013故障
        int m013_status = obj[Define::m013_status].toInt();
        if (m013_status == 3)
        {
            ui->widget_m013_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (m013_status == 4)
        {
            ui->widget_m013_status->setLabelText(Define::STATUS_ONLINE);
        }
        // m014故障
        int m014_status = obj[Define::m014_status].toInt();
        if (m014_status == 3)
        {
            ui->widget_m014_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (m014_status == 4)
        {
            ui->widget_m014_status->setLabelText(Define::STATUS_ONLINE);
        }
        // 气动油低
        int pneumatic_oil_status = obj[Define::pneumatic_oil_status].toInt();
        if (pneumatic_oil_status == 3)
        {
            ui->widget_pneumatic_oil_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (pneumatic_oil_status == 4)
        {
            ui->widget_pneumatic_oil_status->setLabelText(Define::STATUS_ONLINE);
        }
        // 系统运行状态
        int hardware_working_status = obj[Define::hardware_working_status].toInt();
        if (hardware_working_status == 3)
        {
            ui->widget_hardware_working_status->setLabelText(Define::STATUS_RUN);
        }
        else if (hardware_working_status == 4)
        {
            ui->widget_hardware_working_status->setLabelText(Define::STATUS_STOP);
        }
        // 顶升机构状态
        int carbon_up_status = obj[Define::carbon_up_status].toInt();
        if (carbon_up_status == 3)
        {
            ui->widget_carbon_up_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (carbon_up_status == 4)
        {
            ui->widget_carbon_up_status->setLabelText(Define::STATUS_ONLINE);
        }
        // 下降机构状态
        int carbon_down_status = obj[Define::carbon_down_status].toInt();
        if (carbon_down_status == 3)
        {
            ui->widget_carbon_down_status->setLabelText(Define::STATUS_OFFLINE);
        }
        else if (carbon_down_status == 4)
        {
            ui->widget_carbon_down_status->setLabelText(Define::STATUS_ONLINE);
        }
        // 碳块出口信号
        int carbon_exit_safe_status = obj[Define::carbon_exit_safe_status].toInt();
        if (carbon_exit_safe_status == 3)
        {
            ui->widget_carbon_exit_safe_status->setLabelText(Define::STATUS_NOSAFEY);
        }
        else if (carbon_exit_safe_status == 4)
        {
            ui->widget_carbon_exit_safe_status->setLabelText(Define::STATUS_SAFEY);
        }
        // 碳块入口信号
        int carbon_enter_safe_status = obj[Define::carbon_enter_safe_status].toInt();
        if (carbon_enter_safe_status == 3)
        {
            ui->widget_carbon_enter_safe_status->setLabelText(Define::STATUS_NOSAFEY);
        }
        else if (carbon_enter_safe_status == 4)
        {
            ui->widget_carbon_enter_safe_status->setLabelText(Define::STATUS_SAFEY);
        }
        // 安全门信号
        int safe_door = obj[Define::safe_door].toInt();
        if (safe_door == 3)
        {
            ui->widget_safe_door->setLabelText(Define::STATUS_OPENDOOR);
        }
        else if (safe_door == 4)
        {
            ui->widget_safe_door->setLabelText(Define::STATUS_OFFDOOR);
        }
    }
}

void Monitoring_homepage::updateConnectionStatus(bool status)
{
    m_connectionStatus = status;
    if (!m_connectionStatus)
    {
        // 刀具转速
        ui->speedDashboard->setValue(0);
        ui->label_device_pressure_value_5->setText(QString("设备转速：%1").arg(QString::number(0)));
        // 设备气压
        ui->dustPressure->setValue(0);
        ui->label_device_pressure_value_4->setText(QString("设备气压：%1Mpa").arg(QString::number(0)));
        // 收尘气压
        ui->equipmentPressure->setValue(0);
        ui->label_device_pressure_value_3->setText(QString("收尘负压：%1Pa").arg(QString::number(0)));
        // 机械臂状态
        ui->widget_robot_status->setLabelText(Define::STATUS_UNKNOWN);

        // 硬件状态
        ui->widget_hardware_status->setLabelText(Define::STATUS_UNKNOWN);

        // 碳块就位
        ui->widget_carbon_status->setLabelText(Define::STATUS_UNKNOWN);

        // 碳块送走
        ui->widget_carbon_moved->setLabelText(Define::STATUS_UNKNOWN);

        // 放过炭块状态
        ui->widget_pass_carbon_status->setLabelText(Define::STATUS_UNKNOWN);

        // m011故障
        ui->widget_m011_status->setLabelText(Define::STATUS_UNKNOWN);
        // m013故障
        ui->widget_m013_status->setLabelText(Define::STATUS_UNKNOWN);
        // m014故障
        ui->widget_m014_status->setLabelText(Define::STATUS_UNKNOWN);

        // 气动油低
        ui->widget_pneumatic_oil_status->setLabelText(Define::STATUS_UNKNOWN);

        // 系统运行状态
        ui->widget_hardware_working_status->setLabelText(Define::STATUS_UNKNOWN);

        // 顶升机构状态
        ui->widget_carbon_up_status->setLabelText(Define::STATUS_UNKNOWN);

        // 下降机构状态
        ui->widget_carbon_down_status->setLabelText(Define::STATUS_UNKNOWN);

        // 碳块出口信号
        ui->widget_carbon_exit_safe_status->setLabelText(Define::STATUS_UNKNOWN);

        // 碳块入口信号
        ui->widget_carbon_enter_safe_status->setLabelText(Define::STATUS_UNKNOWN);

        // 安全门信号
        ui->widget_safe_door->setLabelText(Define::STATUS_UNKNOWN);
    }
}

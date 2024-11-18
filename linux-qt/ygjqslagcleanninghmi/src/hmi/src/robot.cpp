#include "robot.h"
#include "ui_robot.h"
#include "robotPrivate.h"
#include "customlabelfrom.h"
#include "customDialog.h"
#include "switchbutton.h"

#include "glog/logging.h"
Robot::Robot(QtNode *node_ptr, QWidget *parent) : QWidget(parent),
                                                  ui(new Ui::Robot)

{
    dataPtr.reset(new RobotPrivate());
    ui->setupUi(this);

    dataPtr->qt_node_ = node_ptr;
    // 窗口1
    dataPtr->_vw1 = new QVideoWidget;
    dataPtr->_vw1->installEventFilter(this); // 安装事件过滤器
                                             // 窗口2
    dataPtr->_vw2 = new QVideoWidget;
    dataPtr->_vw2->installEventFilter(this); // 安装事件过滤器

    init_camera();
    init_button_connect_set();
    init_label_from();

    dataPtr->is_fullscreen = false;
    ui->pushButton_switch_1->setVisible(false);
    ui->pushButton_switch_3->setVisible(false);
    // 加载样式
    QFile files(":/qss/robot.qss");
    if (files.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        setStyleSheet(files.readAll());
        files.close();
    }
    // connect
    connect(dataPtr->qt_node_, &QtNode::plc_status_data_info_signal, this, &Robot::recv_plc_status_data_info_slot);
    connect(dataPtr->qt_node_, &QtNode::system_connection_status_signal, this, &Robot::system_connection_status_slot);
    connect(ui->pushButton_mode, &QPushButton::clicked, this, &Robot::on_working_mode_switch);
    connect(ui->pushButton_auto_start, &QPushButton::clicked, this, &Robot::on_auto_working_begin);
}

Robot::~Robot()
{
    dataPtr->_player1.stop();
    dataPtr->_player2.stop();
    delete ui;
}

void Robot::init_camera()
{
    dataPtr->is_camera_url.reserve(4);
    dataPtr->get_camera_info.assign(4, false);

    dataPtr->cur_camera_channel.assign(2, false);

    // 配置文件导入
    QString config_path = QString::fromStdString(dataPtr->qt_node_->config_path);
    QString filename = config_path + "setup.ini";

    if (filename.isEmpty())
    {
        LOG(INFO) << "search config fail! " << std::endl;
    }
    else
    {
        QFile file(filename);
        if (!file.exists())
        {
            LOG(INFO) << "no file context! " << std::endl;
        }
        else
        {
            if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                LOG(INFO) << "no file text! " << std::endl;
            }
            else
            {
                // 2通道
                for (int i = 0; i < 2; i++)
                {
                    if (!file.atEnd())
                    {
                        QByteArray line = file.readLine();
                        dataPtr->is_camera_url.emplace_back(QString::fromUtf8(line));
                        dataPtr->get_camera_info.at(i) = true; // 获取camera信息
                    }
                }
            }
        }
        file.close();
    }

    // 窗口1
    dataPtr->_vw1->setAspectRatioMode(Qt::IgnoreAspectRatio);
    QVBoxLayout *videoVbox1 = new QVBoxLayout;
    videoVbox1->addWidget(dataPtr->_vw1);
    videoVbox1->setContentsMargins(0, 0, 0, 0);
    videoVbox1->setSpacing(0);
    ui->camera1->setLayout(videoVbox1);
    dataPtr->_vw1->setGeometry(0, 0, width(), height());
    // _vw1->show();
    dataPtr->_player1.setVideoOutput(dataPtr->_vw1);
    // 窗口2
    dataPtr->_vw2->setAspectRatioMode(Qt::IgnoreAspectRatio);
    QVBoxLayout *videoVbox2 = new QVBoxLayout;
    videoVbox2->addWidget(dataPtr->_vw2);
    videoVbox2->setContentsMargins(0, 0, 0, 0);
    videoVbox2->setSpacing(0);
    ui->camera2->setLayout(videoVbox2);
    dataPtr->_vw2->setGeometry(0, 0, width(), height());
    // _vw2->show();
    dataPtr->_player2.setVideoOutput(dataPtr->_vw2);

    // 窗口1推流通道0
    set_camera_channel(dataPtr->_player1, 0, 0);
    // 窗口2推流通道1
    set_camera_channel(dataPtr->_player2, 1, 1);
}

void Robot::set_camera_channel(QMediaPlayer &player, int32_t widget, int32_t channel)
{
    if (dataPtr->get_camera_info.at(channel))
    {
        QNetworkRequest requestRtsp;
        requestRtsp.setUrl(QUrl(dataPtr->is_camera_url.at(channel)));
        player.setMedia(requestRtsp);
        player.play();
        dataPtr->cur_camera_channel.at(widget) = !dataPtr->cur_camera_channel.at(widget);
    }
}

void Robot::recv_plc_status_data_info_slot(hmi_msgs::HmiStatus msg)
{
    dataPtr->current_msg = msg;
    // 机器人状态
    if ((msg.robot_connection_status.data == hmi_msgs::RobotConnectionStatus::ONLINE))
    {
        ui->widget_robot->setLabelText(globals::online);
    }
    else
    {
        ui->widget_robot->setLabelText(globals::offline);
    }

    // PLC状态
    if (msg.plc_connection_status.data == hmi_msgs::PLCConnectionStatus::ONLINE)
    {
        ui->widget_PLC->setLabelText(globals::online);
    }
    else
    {
        ui->widget_PLC->setLabelText(globals::offline);
    }
    // 冷却气压
    QString air_pressure =
        QString::number(float(msg.cooler_air_pressure), 'f', 2) + "MPa";
    ui->widget_air_pressure->setLabelText(air_pressure);
    // 风镐气压
    QString pick_pressure = QString::number(float(msg.pneumatic_pick_pressure), 'f', 2) + "MPa";
    ui->widget_pick_pressure->setLabelText(pick_pressure);
    // 控制柜温度
    QString temperature = QString::number(msg.cabinet_temperature, 'f', 2) + "℃";
    ui->widget_control_temperature->setLabelText(temperature);

    // 自动模式
    if (msg.auto_mode_status.data == hmi_msgs::AutoModeStatus::YES)
    {
        ui->pushButton_mode->toggleMode(false);
        ui->stackedWidget->setCurrentIndex(1);
    }
    else
    {
        ui->pushButton_mode->toggleMode(true);
        ui->stackedWidget->setCurrentIndex(0);
    }

    // 一号料口
    if (msg.clean_hole1_enable_state.state == hmi_msgs::TaskEnableState::DISABLED)
    {
        ui->switchButton_8->setProperty("pusbuttonState", "off");
    }
    else
    {
        ui->switchButton_8->setProperty("pusbuttonState", "on");
    }
    ui->switchButton_8->style()->unpolish(ui->switchButton_8);
    ui->switchButton_8->style()->polish(ui->switchButton_8);
    ui->switchButton_8->update();
    // 二号料口
    if (msg.clean_hole2_enable_state.state == hmi_msgs::TaskEnableState::DISABLED)
    {
        ui->switchButton_9->setProperty("pusbuttonState", "off");
    }
    else
    {
        ui->switchButton_9->setProperty("pusbuttonState", "on");
    }
    ui->switchButton_9->style()->unpolish(ui->switchButton_9);
    ui->switchButton_9->style()->polish(ui->switchButton_9);
    ui->switchButton_9->update();

    // 清渣次数
    QString work_count = QString::number((double)msg.clean_count);
    ui->label_gear_num->setText(QString("%1 次").arg(work_count));

    // 东激光雷达
    if (msg.livox1_connection_status.data == hmi_msgs::LivoxConnectionStatus::ONLINE)
    {
        ui->widget_lidar->setLabelText(globals::online);
    }
    else
    {
        ui->widget_lidar->setLabelText(globals::offline);
    }

    // 西激光雷达
    if (msg.livox2_connection_status.data == hmi_msgs::LivoxConnectionStatus::ONLINE)
    {
        ui->widget_lidar_2->setLabelText(globals::online);
    }
    else
    {
        ui->widget_lidar_2->setLabelText(globals::offline);
    }

    // 风镐开关
    if (msg.pneumatic_pick_status.data == hmi_msgs::PneumaticPickStatus::ACTIVE)
    {
        ui->widget_fenggao->setLabelText(globals::on);
    }
    else
    {
        ui->widget_fenggao->setLabelText(globals::closed);
    }

    // 停止按钮
    if (msg.plc_stop_signal_staus.safety_door_1_pause_button)
    { // 北安全门暂停按钮
        ui->widget_dustproof_room->setLightStatus(1, globals::offline);
    }
    else
    {
        ui->widget_dustproof_room->setLightStatus(1, globals::online);
    }

    if (msg.plc_stop_signal_staus.safety_door_2_pause_button)
    { // 西安全门暂停按钮
        ui->widget_dustproof_room->setLightStatus(2, globals::offline);
    }
    else
    {
        ui->widget_dustproof_room->setLightStatus(2, globals::online);
    }

    if (msg.plc_stop_signal_staus.electrical_cabinet_pause_button)
    { // 电气柜暂停按钮
        ui->widget_dustproof_room->setLightStatus(3, globals::offline);
    }
    else
    {
        ui->widget_dustproof_room->setLightStatus(3, globals::online);
    }

    // 安全门使能
    if (msg.safety_door_enable_status.data == hmi_msgs::SafetyDoorsEnableStatus::DISABLED)
    {
        ui->widget_security_door_3->setLabelText(globals::disable);
    }
    else if (msg.safety_door_enable_status.data == hmi_msgs::SafetyDoorsEnableStatus::ENABLED)
    {
        ui->widget_security_door_3->setLabelText(globals::enable);
    }
    else
    {
        ui->widget_security_door_3->setLabelText(globals::dropped);
    }

    // 安全门1
    if (msg.safety_door_1.data == hmi_msgs::SafetyDoorStatus::OPENED)
    {
        ui->widget_security_door_1->securityDoorText(globals::notclosed);
    }
    else if (msg.safety_door_1.data == hmi_msgs::SafetyDoorStatus::UNLOCKED)
    {
        ui->widget_security_door_1->securityDoorText(globals::unlocked);
    }
    else if (msg.safety_door_1.data == hmi_msgs::SafetyDoorStatus::LOCKED)
    {
        ui->widget_security_door_1->securityDoorText(globals::islocked);
    }
    else
    {
        ui->widget_security_door_1->securityDoorText(globals::dropped);
    }
    // 安全门2
    if (msg.safety_door_2.data == hmi_msgs::SafetyDoorStatus::OPENED)
    {
        ui->widget_security_door_2->securityDoorText(globals::notclosed);
    }
    else if (msg.safety_door_2.data == hmi_msgs::SafetyDoorStatus::UNLOCKED)
    {
        ui->widget_security_door_2->securityDoorText(globals::unlocked);
    }
    else if (msg.safety_door_2.data == hmi_msgs::SafetyDoorStatus::LOCKED)
    {
        ui->widget_security_door_2->securityDoorText(globals::islocked);
    }
    else
    {
        ui->widget_security_door_2->securityDoorText(globals::dropped);
    }

    // 系统运行状态
    if (msg.run_mode.data == hmi_msgs::RunModeStatus::LOCAL)
    {
        ui->widget_run_mode->setLabelText("就地");
    }
    else if (msg.run_mode.data == hmi_msgs::RunModeStatus::REMOTE)
    {
        ui->widget_run_mode->setLabelText("远程");
    }
    else if (msg.run_mode.data == hmi_msgs::RunModeStatus::STOP)
    {
        ui->widget_run_mode->setLabelText("停止");
    }
    else if (msg.run_mode.data == hmi_msgs::RunModeStatus::FAULT)
    {
        ui->widget_run_mode->setLabelText("故障");
    }
    else
    {
        ui->widget_run_mode->setLabelText("未知");
    }

    // 机器人执行器
    if (msg.robot_controller_status.data == hmi_msgs::RobotControllerStatus::STOPPED)
    {
        ui->widget_position->setLabelText(globals::stop);
        ui->widget_position->setbreathingLight(true);
        setBackgroundSoud(true);
    }
    else if (msg.robot_controller_status.data == hmi_msgs::RobotControllerStatus::RUNNING)
    { // 运行中
        ui->widget_position->setLabelText(globals::running);
        ui->widget_position->setbreathingLight(false);
        setBackgroundSoud(false);
    }
}

void Robot::system_connection_status_slot(bool falg)
{
    dataPtr->is_connect = falg;
    systemDisplayState();
}

void Robot::init_button_connect_set()
{
    dataPtr->mapper = new QSignalMapper(this);

    dataPtr->mapper->setMapping(ui->switchButton_8, ui->switchButton_8);
    dataPtr->mapper->setMapping(ui->switchButton_9, ui->switchButton_9);
    connect(ui->switchButton_8, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));
    connect(ui->switchButton_9, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));

    dataPtr->mapper->setMapping(ui->pushButton_p1, ui->pushButton_p1);
    dataPtr->mapper->setMapping(ui->pushButton_p2, ui->pushButton_p2);
    dataPtr->mapper->setMapping(ui->pushButton_p0, ui->pushButton_p0);
    connect(ui->pushButton_p1, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));
    connect(ui->pushButton_p2, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));
    connect(ui->pushButton_p0, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));

    dataPtr->mapper->setMapping(ui->pushButton_start_stop, ui->pushButton_start_stop);
    dataPtr->mapper->setMapping(ui->pushButton_reset, ui->pushButton_reset);
    connect(ui->pushButton_start_stop, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));
    connect(ui->pushButton_reset, SIGNAL(clicked()), dataPtr->mapper, SLOT(map()));

    // mapper
    connect(dataPtr->mapper, SIGNAL(mapped(QWidget *)), this, SLOT(switchButton_clicked(QWidget *)));
    connect(dataPtr->mapper, SIGNAL(mapped(QWidget *)), this, SLOT(working_mode_switch(QWidget *)));
    connect(dataPtr->mapper, SIGNAL(mapped(QWidget *)), this, SLOT(program_state_switch(QWidget *)));
}

void Robot::init_label_from()
{
    // 机器人状态
    ui->widget_robot->setLabelIcon(QString("background: url(:/resource/svg/机器人位置-icon.svg)"));
    ui->widget_robot->setLabelTitle(QString("机器人状态"));
    ui->widget_robot->setLabelText(globals::offline);
    // PLC
    ui->widget_PLC->setLabelIcon(QString("background: url(:/resource/svg/PLC状态-icon.svg)"));
    ui->widget_PLC->setLabelTitle(QString("PLC"));
    ui->widget_PLC->setLabelText(globals::offline);
    // 机器人执行器
    ui->widget_position->setLabelIcon(QString("background: url(:/resource/svg/机器人状态-icon.svg)"));
    ui->widget_position->setLabelTitle(QString("机器人执行器"));
    ui->widget_position->setLabelText(globals::offline);

    // 风稿
    ui->widget_fenggao->setLabelIcon(QString("background: url(:/resource/svg/风镐-icon.svg)"));
    ui->widget_fenggao->setLabelTitle(QString("风稿"));
    ui->widget_fenggao->setLabelText(globals::closed);

    // 机柜温度
    ui->widget_control_temperature->setLabelIcon(QString("background: url(:/resource/svg/控制柜温度-icon.svg)"));
    ui->widget_control_temperature->setLabelTitle(QString("机柜温度"));
    ui->widget_control_temperature->setLabelText(globals::toBeDetected);

    // 冷却气压
    ui->widget_air_pressure->setLabelIcon(QString("background: url(:/resource/svg/气压值-icon.svg)"));
    ui->widget_air_pressure->setLabelTitle(QString("冷却气压"));
    ui->widget_air_pressure->setLabelText(globals::toBeDetected);

    // 风镐气压
    ui->widget_pick_pressure->setLabelIcon(QString("background: url(:/resource/svg/气压值-icon.svg)"));
    ui->widget_pick_pressure->setLabelTitle(QString("风镐气压"));
    ui->widget_pick_pressure->setLabelText(globals::toBeDetected);

    // 北安全门停止按钮
    ui->widget_dustproof_room->setLabelIcon(QString("background: url(:/resource/svg/钳制器状态-icon.svg)"));
    ui->widget_dustproof_room->setLabelTitle(QString("外部停止"));
    ui->widget_dustproof_room->setToolTipStr("北安全门停止按钮", "西安全门停止按钮", "电器柜停止按钮");
    ui->widget_dustproof_room->setLightStatus(0, globals::dropped);

    // 安全门使能
    ui->widget_security_door_3->setLabelIcon(QString("background: url(:/resource/svg/雷达仓盖-icon.svg)"));
    ui->widget_security_door_3->setLabelTitle(QString("安全门使能"));
    ui->widget_security_door_3->setLabelText(globals::abnormal);
    // 北安全门
    ui->widget_security_door_1->setLabelIcon(QString("background: url(:/resource/svg/电气室急停按钮-icon.svg)"));
    ui->widget_security_door_1->setLabelTitle(QString("北安全门"));
    ui->widget_security_door_1->securityDoorText(globals::abnormal);
    // 西安全门
    ui->widget_security_door_2->setLabelIcon(QString("background: url(:/resource/svg/电气室急停按钮-icon.svg)"));
    ui->widget_security_door_2->setLabelTitle(QString("西安全门"));
    ui->widget_security_door_2->securityDoorText(globals::abnormal);

    // 东激光雷达
    ui->widget_lidar->setLabelIcon(QString("background: url(:/resource/svg/雷达状态-icon.svg)"));
    ui->widget_lidar->setLabelTitle("东激光雷达");
    ui->widget_lidar->setLabelText(globals::online);

    // 西激光雷达
    ui->widget_lidar_2->setLabelIcon(QString("background: url(:/resource/svg/雷达状态-icon.svg)"));
    ui->widget_lidar_2->setLabelTitle("西激光雷达");
    ui->widget_lidar_2->setLabelText(globals::online);

    // 系统运行状态
    ui->widget_run_mode->setLabelIcon(QString("background: url(:/resource/svg/机器人位置-icon.svg)"));
    ui->widget_run_mode->setLabelTitle(QString("系统运行状态"));
    ui->widget_run_mode->setLabelText(globals::droppedDo);

    setBackgroundSoud(true);
}

bool Robot::messageDialog(const QString &str, globals::DialogType type)
{
    CustomDialog Dialog(str, type, this);
    if (Dialog.exec() == QDialog::Accepted)
    {
        return true;
    }
    return false;
}

void Robot::systemDisplayState()
{
    if (!dataPtr->is_connect)
    {
        // 机器人状态
        ui->widget_robot->setLabelText(globals::dropped);
        // PLC
        ui->widget_PLC->setLabelText(globals::dropped);
        // 机器人执行器
        ui->widget_position->setLabelText(globals::droppedDo);
        ui->widget_position->setbreathingLight(false);
        // 风稿
        ui->widget_fenggao->setLabelText(globals::dropped);
        // 机柜温度
        ui->widget_control_temperature->setLabelText(globals::droppedDo);
        // 冷却气压
        ui->widget_air_pressure->setLabelText(globals::droppedDo);
        // 风镐气压
        ui->widget_pick_pressure->setLabelText(globals::droppedDo);
        // 北安全门停止按钮
        ui->widget_dustproof_room->setLightStatus(0, globals::dropped);
        // 安全门使能
        ui->widget_security_door_3->setLabelText(globals::dropped);
        // 北安全门
        ui->widget_security_door_1->securityDoorText(globals::dropped);
        // 西安全门
        ui->widget_security_door_2->securityDoorText(globals::dropped);
        // 激光雷达
        ui->widget_lidar->setLabelText(globals::dropped);
        ui->widget_lidar_2->setLabelText(globals::dropped);
        // 系统运行状态
        ui->widget_run_mode->setLabelText(globals::droppedDo);
        setBackgroundSoud(true);
    }
}

void Robot::set_label_text(QLabel &label_text, QString str)
{
    label_text.setText(str);
}

// 窗口1相机切换
void Robot::on_pushButton_switch_1_clicked()
{
    // 当前通道channel0，切换成channel2
    if (dataPtr->cur_camera_channel.at(0))
    {
        dataPtr->_player1.stop();
        // 窗口1推流通道2
        set_camera_channel(dataPtr->_player1, 0, 2);
    }
    else
    {
        // 当前channel2或无显示 切换channel0
        if (dataPtr->get_camera_info.at(0)) // channel0为true，说明已经切换过了 当前通道为channel2
        {
            dataPtr->_player1.stop();
        }
        // 窗口1推流通道0
        set_camera_channel(dataPtr->_player1, 0, 0);
    }
}

// 窗口2相机切换
void Robot::on_pushButton_switch_3_clicked()
{
    // 当前通道channel1，切换成channel3
    if (dataPtr->cur_camera_channel.at(1))
    {
        dataPtr->_player2.stop();
        // 窗口2推流通道3
        set_camera_channel(dataPtr->_player2, 1, 3);
    }
    else
    {
        // 当前channel3或无显示 切换channel1
        if (dataPtr->get_camera_info.at(1)) // channel1为true 说明已经切换过了 当前通道为channel3
        {
            dataPtr->_player2.stop();
        }
        // 窗口2推流通道1
        set_camera_channel(dataPtr->_player2, 1, 1);
    }
}

void Robot::switchButton_clicked(QWidget *widget)
{
    if (!dataPtr->is_connect)
    {
        LOG(INFO) << "远程未连接！";
        return;
    }
    auto button = dynamic_cast<QPushButton *>(widget);
    if (button == nullptr)
    {
        return;
    }
    if (button->objectName() == "switchButton_8")
    {
        if (dataPtr->current_msg.clean_hole1_enable_state.state == hmi_msgs::TaskEnableState::DISABLED)
        {
            if (messageDialog("东下料口给予使能"))
            {
                dataPtr->log_content = "下发东下料口给予使能指令";
                dataPtr->qt_node_->set_task_enable(1, true);
            }
        }
        else
        {
            if (messageDialog("东下料口取消使能"))
            {
                dataPtr->log_content = "下发东下料口取消使能指令";
                dataPtr->qt_node_->set_task_enable(1, false);
            }
        }
    }
    else if (button->objectName() == "switchButton_9")
    {
        if (dataPtr->current_msg.clean_hole2_enable_state.state == hmi_msgs::TaskEnableState::DISABLED)
        {
            if (messageDialog("西下料口给予使能"))
            {
                dataPtr->log_content = "下发西下料口给予使能指令";
                dataPtr->qt_node_->set_task_enable(2, true);
            }
        }
        else
        {
            if (messageDialog("西下料口取消使能"))
            {
                dataPtr->log_content = "下发西下料口取消使能指令";
                dataPtr->qt_node_->set_task_enable(2, false);
            }
        }
    }

    dataPtr->qt_node_->save_config_info(dataPtr->qt_node_->config_path + "config_task.yaml");
}

void Robot::working_mode_switch(QWidget *widget)
{
    auto button = dynamic_cast<QPushButton *>(widget);
    if (button == nullptr)
    {
        return;
    }
    if ("pushButton_auto_start" == button->objectName())
    {
        if (1) // 自动模式
        {
            if (messageDialog("开始自动模式"))
            {
                dataPtr->qt_node_->set_working_mode(true);
            }
        }
        else
        { // 结束自动
            if (messageDialog("结束自动模式"))
            {
                dataPtr->qt_node_->set_working_mode(false);
            }
        }
    }

    else if (button->objectName() == "pushButton_p1")
    {
        if (messageDialog("前往东下料口"))
        {
            dataPtr->qt_node_->set_manualCommand(3);
        }
    }
    else if (button->objectName() == "pushButton_p2")
    {
        if (messageDialog("前往西入料口"))
        {
            dataPtr->qt_node_->set_manualCommand(4);
        }
    }
    else if (button->objectName() == "pushButton_p0")
    {
        if (messageDialog("安全撤回"))
        {
            dataPtr->qt_node_->set_system_back();
        }
    }
}

void Robot::on_working_mode_switch()
{
    if (!dataPtr->is_connect)
    {
        LOG(INFO) << "远程未连接！";
        return;
    }
    if (dataPtr->current_msg.auto_mode_status.data == hmi_msgs::AutoModeStatus::YES)
    {
        dataPtr->qt_node_->set_working_mode(false);
    }
    else
    {
        dataPtr->qt_node_->set_working_mode(true);
    }
}

void Robot::on_auto_working_begin()
{
    if (!dataPtr->is_connect)
    {
        LOG(INFO) << "远程未连接！";
        return;
    }
    dataPtr->qt_node_->set_auto_working_begin(true);
}

void Robot::program_state_switch(QWidget *widget)
{
    auto button = dynamic_cast<QPushButton *>(widget);
    if (button == nullptr)
    {
        return;
    }
    if (button->objectName() == "pushButton_reset")
    { // 复位机器人
        if (messageDialog("复位机器人"))
        {
            dataPtr->log_content = "下发复位机器人指令";
            dataPtr->qt_node_->set_system_reset();
            sleep(3);
            system("ssh ubuntu@192.168.10.11 /home/ubuntu/workspace/scripts/restart_robot_app.sh");
        }
    }

    if (!dataPtr->is_connect)
    {
        LOG(INFO) << "远程未连接！";
        return;
    }

    if (button->objectName() == "pushButton_start_stop")
    {
        // 继续运行
        if (messageDialog("继续运行"))
        {
            dataPtr->qt_node_->set_system_continue();
        }
    }
}

bool Robot::eventFilter(QObject *obj, QEvent *event)
{
    auto handleFullscreen = [this](QWidget *vw, const QString &cameraName)
    {
        if (!dataPtr->is_fullscreen)
        {
            // qDebug() << cameraName << "进入全屏";
            dataPtr->is_fullscreen = true;
            dataPtr->widget_rect_ = vw->geometry();
            vw->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
            vw->setFocus();
            vw->showFullScreen();
        }
        else
        {
            // qDebug() << cameraName << "退出全屏";
            dataPtr->is_fullscreen = false;
            vw->setWindowFlags(Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::WindowMinMaxButtonsHint |
                               Qt::WindowCloseButtonHint);
            vw->showNormal();
            vw->setGeometry(dataPtr->widget_rect_);
        }
    };

    auto handleKeyPress = [this](QWidget *vw)
    {
        dataPtr->is_fullscreen = false;
        vw->setWindowFlags(Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::WindowMinMaxButtonsHint |
                           Qt::WindowCloseButtonHint);
        vw->showNormal();
        vw->setGeometry(dataPtr->widget_rect_);
    };

    // 根据对象判断是哪个摄像头，并处理事件
    if (obj == dataPtr->_vw1 || obj == dataPtr->_vw2)
    {
        QWidget *vw = static_cast<QWidget *>(obj);
        QString cameraName = (vw == dataPtr->_vw1) ? "摄像头1" : "摄像头2";

        if (event->type() == QEvent::MouseButtonDblClick)
        {
            handleFullscreen(vw, cameraName);
            return true;
        }
        else if (event->type() == QEvent::KeyPress)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape)
            {
                handleKeyPress(vw);
                return true;
            }
        }
    }
    return false; // 将事件传递给基类处理
}

void Robot::setBackgroundSoud(bool falg)
{
    background_sound(falg && dataPtr->sound_flag);
}

void Robot::setBackgroundSoudFalg(bool falg)
{
    dataPtr->sound_flag = falg;
    background_sound(falg);
}

void Robot::background_sound(bool falg)
{
    if (falg)
    {
        if (dataPtr->sound->isFinished())
        {
            dataPtr->sound->setLoops(QSound::Infinite);
            dataPtr->sound->play();
        }
    }
    else
    {
        if (!dataPtr->sound->isFinished())
        {
            dataPtr->sound->stop();
        }
    }
}

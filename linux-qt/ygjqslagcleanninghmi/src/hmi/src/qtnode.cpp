#include "qtnode.h"
#include <QDebug>
#include "globals.h"
#include <QDateTime>

#include "glog/logging.h"
QtNode::QtNode(int argc, char **argv, const std::string &node_name) : argc_(argc), argv_(argv), node_name_(node_name)
{
  qRegisterMetaType<hmi_msgs::HmiStatus>("hmi_msgs::HmiStatus");
  qRegisterMetaType<QVector<QString>>("QVector<QString>");

  QTimer *timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &QtNode::checkPlcStatusDataCallback);
  timer->start(1000); // 设置计时器每1000毫秒触发一次
  // 初始化最后一次调用时间
  lastCallbackTime = QDateTime::currentDateTime();
}

QtNode::~QtNode()
{
  LOG(INFO) << "QtNode::~QtNode";
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

void QtNode::init()
{
  ros::init(argc_, argv_, node_name_);
  ros::NodeHandle nh("~");

  nh.getParam("/config_path", config_path);

  node_ptr_ = std::make_shared<ros::NodeHandle>("~");
  parse_yaml_config(config_path);

  control_command_cli_ = node_ptr_->serviceClient<hmi_msgs::ControlCommandSrv>("/hmi/control_command");
  plc_data_sub_ = node_ptr_->subscribe<hmi_msgs::HmiStatus>("/hmi/status", 1, &QtNode::plc_status_data_callback, this);
  querySystem_logs_cli_ = node_ptr_->serviceClient<hmi_msgs::QuerySystemLogsSrv>("/hmi/query_system_logs");
  queryOperation_logs_cli_ = node_ptr_->serviceClient<hmi_msgs::QueryOperationLogsSrv>("/hmi/query_operation_logs");
  singleSystem_log_sub_ = node_ptr_->subscribe<hmi_msgs::SystemLogInfo>("/hmi/latest_system_log", 1, &QtNode::plc_singleSystem_log_callback, this);
  manualCommand_pub_ = node_ptr_->advertise<hmi_msgs::ManualCommand>("/hmi/manual_command", 1);

  system_params_cli_ = node_ptr_->serviceClient<hmi_msgs::SystemParamsSrv>("/hmi/system_params");
}

void QtNode::run() { ros::spin(); }

// 读取配置文件
void QtNode::parse_yaml_config(std::string config_path)
{
  YAML::Node config_task = YAML::LoadFile(config_path + "/config_task.yaml");
  m_page_row_count = config_task["page_row_count"].as<int>();
  m_operation_log_row_count = config_task["operation_log_row_count"].as<int>();
}

// 保存参数
void QtNode::save_config_info(const std::string file)
{
  YAML::Node task_node;

  task_node["page_row_count"] = m_page_row_count;
  task_node["operation_log_row_count"] = m_operation_log_row_count;
  std::ofstream fout(file);
  fout << task_node;
  fout.close();
}

void QtNode::plc_singleSystem_log_callback(const hmi_msgs::SystemLogInfo::ConstPtr &msg)
{
  QVector<QString> log;
  log << globals::MessageTypeMap[(int)msg->level] << QDateTime::fromSecsSinceEpoch(msg->time.toSec()).toString("yyyy-MM-dd hh:mm:ss")
      << QString::fromStdString(msg->message);
  emit plc_singleSystem_log_signal(log);
}

QVector<QVector<QString>> QtNode::get_system_logs()
{
  QVector<QVector<QString>> returnLogs;
  // 创建服务的请求
  hmi_msgs::QuerySystemLogsSrv srv;
  // 调用服务
  if (querySystem_logs_cli_.call(srv))
  {
    auto logs = srv.response.systemlogs;
    for (int i = 0; i < logs.size(); i++)
    {
      QVector<QString> log;
      log << globals::MessageTypeMap[(int)logs[i].level] << QDateTime::fromSecsSinceEpoch(logs[i].time.toSec()).toString("yyyy-MM-dd hh:mm:ss")
          << QString::fromStdString(logs[i].message);
      returnLogs.push_back(log);
    }
  }

  return returnLogs;
}

QVector<QVector<QString>> QtNode::get_operation_logs()
{
  QVector<QVector<QString>> operatLogs;
  // 创建服务的请求
  hmi_msgs::QueryOperationLogsSrv srv;
  // 调用服务
  if (queryOperation_logs_cli_.call(srv))
  {
    auto logs = srv.response.operationlogs;
    for (int i = 0; i < logs.size(); i++)
    {
      QVector<QString> log;
      log.append(QDateTime::fromSecsSinceEpoch(logs[i].time.toSec()).toString("yyyy-MM-dd hh:mm:ss"));
      log.append(QString::fromStdString(logs[i].message));
      operatLogs.push_back(log);
    }
  }
  return operatLogs;
}

void QtNode::set_system_params(const QString &str, const QStringList list)
{
  // 创建srv的请求和响应对象
  hmi_msgs::SystemParamsSrv srv;
  if (str == globals::systemParamsLoad)
  {
    srv.request.require_type.data = hmi_msgs::SystemParamsSrvType::LOAD;
    if (system_params_cli_.call(srv))
    {
      emit timer_signal(QString::number(srv.response.work_timing_interval));
      emit coordinate_signal(QStringList() << QString::number(srv.response.workpiece_offset.x)
                                           << QString::number(srv.response.workpiece_offset.y)
                                           << QString::number(srv.response.workpiece_offset.z));
    }
    return;
  }

  if (str == globals::workTimeUpdate)
  {
    srv.request.require_type.data = hmi_msgs::SystemParamsSrvType::UPDATE_WORK_INTERVAL;
    if (list.size() == 1)
      srv.request.work_timing_interval = list.at(0).toFloat();
  }
  else if (str == globals::toolOffsetUpdate)
  {
    if (list.size() == 3)
    {
      srv.request.workpiece_offset.x = list.at(0).toFloat();
      srv.request.workpiece_offset.y = list.at(1).toFloat();
      srv.request.workpiece_offset.z = list.at(2).toFloat();
    }
    srv.request.require_type.data = hmi_msgs::SystemParamsSrvType::UPDATE_WORKPIECE_OFFSET;
  }

  if (system_params_cli_.call(srv))
  {
  }
}

// ROS图像消息转换为OpenCV图像
void QtNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try // 对错误异常进行捕获，检查数据的有效性
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    char img_name[256];
    sprintf(img_name, "%simg/%s.jpg", config_path.data(),
            QDateTime::currentDateTime().toString("yyyy_MM_dd.hh:mm:ss").toLatin1().data());
    // 保存图像到文件
    imwrite(img_name, cv_ptr->image);
  }
  catch (cv_bridge::Exception &e) // 异常处理
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void QtNode::set_working_mode(bool falg)
{
  hmi_msgs::ControlCommandSrv srv;
  if (falg)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::AUTO_MODE_ON;
  }
  else
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::AUTO_MODE_OFF;
  }

  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::set_auto_working_begin(bool falg)
{
  hmi_msgs::ControlCommandSrv srv;
  if (falg)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::AUTO_MODE_START_MOVE;
  }

  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::set_task_enable(int index, bool enable)
{
  hmi_msgs::ControlCommandSrv srv;
  if (index == 1 && enable == true)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::ENABLE_CLEAN_HOLE1;
  }
  else if (index == 2 && enable == true)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::ENABLE_CLEAN_HOLE2;
  }
  else if (index == 1 && enable == false)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::DISABLE_CLEAN_HOLE1;
  }
  else if (index == 2 && enable == false)
  {
    srv.request.control_command.cmd = hmi_msgs::ControlCommand::DISABLE_CLEAN_HOLE2;
  }
  else
  {
    std::cerr << "index or enable is wrong" << std::endl;
  }

  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::set_system_reset()
{
  hmi_msgs::ControlCommandSrv srv;
  srv.request.control_command.cmd = hmi_msgs::ControlCommand::SYSETM_RESET;
  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::set_system_back()
{
  hmi_msgs::ControlCommandSrv srv;
  srv.request.control_command.cmd = hmi_msgs::ControlCommand::SAFETY_BACK;
  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::set_manualCommand(int index)
{
  hmi_msgs::ManualCommand command;
  if (index == 3)
  {
    command.cmd = hmi_msgs::ManualCommand::CLEAN_HOLE_1;
    manualCommand_pub_.publish(command);
  }
  else if (index == 4)
  {
    command.cmd = hmi_msgs::ManualCommand::CLEAN_HOLE_2;
    manualCommand_pub_.publish(command);
  }
}

void QtNode::set_system_continue()
{
  hmi_msgs::ControlCommandSrv srv;
  srv.request.control_command.cmd = hmi_msgs::ControlCommand::CONTINUE_RUN;

  if (control_command_cli_.call(srv))
  {
    LOG(INFO) << "send command success" << std::endl;
  }
  else
  {
    LOG(INFO) << "send command failed" << std::endl;
  }
}

void QtNode::plc_status_data_callback(const hmi_msgs::HmiStatus::ConstPtr &msg)
{
  emit plc_status_data_info_signal(*msg);
  // 更新最后一次调用时间
  lastCallbackTime = QDateTime::currentDateTime();
}

void QtNode::checkPlcStatusDataCallback()
{
  QDateTime currentTime = QDateTime::currentDateTime();
  int elapsed = lastCallbackTime.msecsTo(currentTime);
  // 检查是否在过去的一段时间内回调函数被调用
  emit system_connection_status_signal(elapsed > 2000 ? false : true);
}
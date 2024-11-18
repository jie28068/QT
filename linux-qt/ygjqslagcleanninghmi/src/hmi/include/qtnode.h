#ifndef WATTMAN_HMI_QTNODE_H
#define WATTMAN_HMI_QTNODE_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdint.h>
#include <QTimer>
#include <QDateTime>
#include <QVector>
#include <QDebug>
#include <QMetaType>
#include <QThread>
#include <QString>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <geometry_msgs/Point.h>

#include "hmi_msgs/HmiStatus.h"
#include "hmi_msgs/ControlCommandSrv.h"
#include "hmi_msgs/QuerySystemLogsSrv.h"
#include "hmi_msgs/QueryOperationLogsSrv.h"
#include "hmi_msgs/ManualCommand.h"
#include "hmi_msgs/SystemParamsSrv.h"

class QtNode : public QThread
{
  Q_OBJECT

public:
  QtNode(int argc, char **argv, const std::string &node_name);
  ~QtNode();

  void init();
  void run();

  /// @brief 设置工作模式
  /// @param falg
  void set_working_mode(bool falg);
  /// @brief 设置自动模式
  /// @param falg
  void set_auto_working_begin(bool falg);
  /// @brief 设置料口工作开关
  /// @param index 几号料口
  /// @param enable 开/关
  void set_task_enable(int index, bool enable);
  /// @brief 设置系统复位
  void set_system_reset();
  /// @brief 设置安全撤回
  void set_system_back();
  /// @brief 手动命令几号料口
  void set_manualCommand(int index);
  /// @brief 设置继续运行
  void set_system_continue();

  void plc_status_data_callback(const hmi_msgs::HmiStatus::ConstPtr &msg);
  void save_config_info(const std::string file);
  void plc_singleSystem_log_callback(const hmi_msgs::SystemLogInfo::ConstPtr &msg);

  QVector<QVector<QString>> get_system_logs();
  QVector<QVector<QString>> get_operation_logs();

  /// @brief 设置系统参数
  /// @param str
  void set_system_params(const QString &str, const QStringList list);

private slots:
  void checkPlcStatusDataCallback();

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  // 加载配置文件
  void parse_yaml_config(std::string config_path);
Q_SIGNALS:
  void plc_status_data_info_signal(hmi_msgs::HmiStatus plc_status_data_info);
  void livox_cloud_signal(QString topic);
  /// 跟踪系统连接状态
  void system_connection_status_signal(bool);
  void plc_singleSystem_log_signal(QVector<QString> str);

  // 时间间隔
  void timer_signal(const QString &);
  /// 坐标系
  void coordinate_signal(const QStringList &);

private:
  std::shared_ptr<ros::NodeHandle> node_ptr_;
  int argc_;
  char **argv_;
  std::string node_name_;
  /// @brief 跟踪最后一次调用时间
  QDateTime lastCallbackTime;

  ros::ServiceClient control_command_cli_;
  ros::ServiceClient querySystem_logs_cli_;
  ros::Subscriber singleSystem_log_sub_;
  ros::ServiceClient queryOperation_logs_cli_;
  ros::Subscriber plc_data_sub_;
  ros::Publisher manualCommand_pub_;
  ros::ServiceClient system_params_cli_;

public:
  // 配置文件地址
  std::string config_path;
  // 记录当前系统日志显示行数
  int m_page_row_count = 5;
  // 记录当前操作日志显示行数
  int m_operation_log_row_count = 5;
};

#endif // WATTMAN_HMI_QTNODE_H

#ifndef ROBOT_H
#define ROBOT_H

#include <QWidget>
#include <QLabel>

#include "globals.h"
#include "qtnode.h"

namespace Ui
{
    class Robot;
}

class QtNode;
class RobotPrivate;
class QMediaPlayer;

class Robot : public QWidget
{
    Q_OBJECT
public:
    explicit Robot(QtNode *node_ptr, QWidget *parent = 0);
    ~Robot();
    bool eventFilter(QObject *obj, QEvent *event);
    /// @brief 设置背景音乐开关
    void setBackgroundSoud(bool falg);
    void setBackgroundSoudFalg(bool falg);
    void background_sound(bool falg);

private slots:
    void on_pushButton_switch_1_clicked();
    void on_pushButton_switch_3_clicked();
    /// @brief 作业状态切换事件集
    void switchButton_clicked(QWidget *widget);
    /// @brief 工作模式切换事件集
    void working_mode_switch(QWidget *widget);
    /// @brief 工作模式切换
    void on_working_mode_switch();
    /// @brief 自动模式开始
    void on_auto_working_begin();
    /// @brief 程序状态切换事件集
    void program_state_switch(QWidget *widget);
    /// @brief 发送来的数据集设置状态
    void recv_plc_status_data_info_slot(hmi_msgs::HmiStatus msg);
    /// @brief 上下位机连接状态
    void system_connection_status_slot(bool falg);

private:
    void init_camera();                                                             // 初始化获得camera信息                                                     // 初始化清渣作业次数
    void set_camera_channel(QMediaPlayer &player, int32_t widget, int32_t channel); // 设置camera通道推流
    void set_label_text(QLabel &label_text, QString str);                           // 设置系统数据具体值

    /// @brief 初始化按钮事件集
    void init_button_connect_set();
    /// @brief 初始化自定义label组件
    void init_label_from();
    /// @brief 消息弹窗
    bool messageDialog(const QString &str, globals::DialogType type = globals::DialogType::ConfirmDialog);
    /// @brief 系统信息显示状态，离线时全部放置灰
    void systemDisplayState();

private:
    Ui::Robot *ui;
    QScopedPointer<RobotPrivate> dataPtr;
};

#endif // MainWindow_H

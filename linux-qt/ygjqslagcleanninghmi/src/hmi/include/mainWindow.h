#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QLabel>
#include "robot.h"
#include "image.h"
// #include "TcpIf/dataserv.h"
#include <QWindowStateChangeEvent>
#include "qtnode.h"

class DataTableWidget;
namespace Ui
{
    class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();

private:
    QtNode *qt_node_ = nullptr;
    Ui::MainWindow *ui;
    Robot *ui_page1;
    DataTableWidget *tableOperationLog;
    DataTableWidget *tableSystemLog;
    Image *ui_page5;
    Qt::WindowStates m_OldWindowState = Qt::WindowNoState;
    Qt::WindowStates m_WindowState = Qt::WindowNoState;
    int error_code = 0; // 记录发送的机器人故障码
    void initQtNode();
    void initSound();
    bool is_sound = false;

private slots:
    void update_system_time_slot();
    void changeEvent(QEvent *event);

    void on_pushButton_minimize_clicked();
    void on_pushButton_close_clicked();

    void on_pushButton_page1_clicked();
    void on_pushButton_page2_clicked();
    void on_pushButton_page3_clicked();
    void on_pushButton_page4_clicked();
    void on_pushButton_page5_clicked();
    void plc_singleSystem_log_slot(QVector<QString> str);
    void setBackgroundSound();
};

#endif // WIDGET_H

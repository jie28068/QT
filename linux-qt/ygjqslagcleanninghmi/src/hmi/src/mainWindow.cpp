#include "mainWindow.h"
#include "ui_mainWindow.h"
#include <QThread>
#include <QDateTime>
#include <QTimer>
#include "clickableLabel.h"
#include "dataTableWidget.h"
#include "globals.h"

#include "parameterdialog.h"
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QWidget(parent),
                                                                 ui(new Ui::MainWindow)
{
    qt_node_ = new QtNode(argc, argv, "hmi");
    initQtNode();
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    setWindowState(Qt::WindowFullScreen);
    // 窗口定义
    ui_page1 = new Robot(qt_node_);
    tableOperationLog = new DataTableWidget(qt_node_, globals::operationLog);
    tableSystemLog = new DataTableWidget(qt_node_, globals::systemLog);
    ui->stackedWidget->addWidget(ui_page1);
    ui->stackedWidget->addWidget(tableOperationLog);
    ui->stackedWidget->addWidget(tableSystemLog);
    // 默认为MainWindow页面
    ui->stackedWidget->setCurrentIndex(0);

    // Timer update time
    QTimer *timer = new QTimer(this);
    timer->setInterval(1000);
    connect(timer, &QTimer::timeout, this, &MainWindow::update_system_time_slot);
    timer->start();

    // connect
    connect(qt_node_, &QtNode::plc_singleSystem_log_signal, this, &MainWindow::plc_singleSystem_log_slot);
    connect(ui->label_err, &ClickableLabel::clicked, ui->pushButton_page3, &QPushButton::click);
    connect(ui->pushButton_page5, &QPushButton::clicked, this, [=]()
            {
                ParameterDialog *dialog = new ParameterDialog(qt_node_, this);
                dialog->exec();
                ui->pushButton_page1->click(); });
    connect(ui->pushButton_vode, &QPushButton::clicked, this, &MainWindow::setBackgroundSound);
    initSound();
}

MainWindow::~MainWindow()
{
    delete qt_node_;
    system("rosnode kill /hmi_qt");
    delete ui;
}

void MainWindow::initQtNode()
{
    qt_node_->init();
    qt_node_->start();
}

void MainWindow::initSound()
{
    YAML::Node config_task = YAML::LoadFile(qt_node_->config_path + "/config_sound.yaml");
    is_sound = config_task["brackground_sound"].as<bool>();
    if (!is_sound)
    {
        ui->pushButton_vode->setStyleSheet(R"(
                                    border-radius: 50%;
                                    color: rgb(255, 255, 255);
                                    border-image: url(:/resource/sound/静音.svg);)");
    }
    else
    {
        ui->pushButton_vode->setStyleSheet(R"(
                                    border-radius: 50%;
                                    color: rgb(255, 255, 255);
                                    border-image: url(:/resource/sound/音量.svg);)");
    }
    ui_page1->setBackgroundSoudFalg(is_sound);
}

void MainWindow::update_system_time_slot()
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui->label_time_init->setText(dateTime.toString("yyyy-MM-dd hh:mm:ss"));
}

void MainWindow::changeEvent(QEvent *event)
{
    if (event->type() == QEvent::WindowStateChange)
    {
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

void MainWindow::on_pushButton_minimize_clicked()
{
    m_OldWindowState = this->windowState();
    this->setWindowState(Qt::WindowMinimized);
}

void MainWindow::on_pushButton_close_clicked()
{
    this->close();
}

void MainWindow::on_pushButton_page1_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::on_pushButton_page2_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
    tableOperationLog->loadData();
}

void MainWindow::on_pushButton_page3_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
    tableSystemLog->loadData();
}

void MainWindow::on_pushButton_page4_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
}

void MainWindow::on_pushButton_page5_clicked()
{
}

void MainWindow::plc_singleSystem_log_slot(QVector<QString> str)
{
    if (str.size() < 3 || str.isEmpty())
        return;
    if (globals::MessageTypeMap[str[0].toInt()] == globals::normalMessage)
    {
        ui->label_err->setStatus(ClickableLabel::Normal);
    }
    else if (globals::MessageTypeMap[str[0].toInt()] == globals::warningMessage)
    {
        ui->label_err->setStatus(ClickableLabel::Warning);
    }
    else if (globals::MessageTypeMap[str[0].toInt()] == globals::errorMessage)
    {
        ui->label_err->setStatus(ClickableLabel::Error);
    }
    ui->label_err->setText(QString("[%1] %2 %3").arg(globals::MessageTypeMap[str[0].toInt()]).arg(str[1]).arg(str[2]));
}

void MainWindow::setBackgroundSound()
{
    if (is_sound)
    {
        ui->pushButton_vode->setStyleSheet(R"(
                                    border-radius: 50%;
                                    color: rgb(255, 255, 255);
                                    border-image: url(:/resource/sound/静音.svg);)");
    }
    else
    {
        ui->pushButton_vode->setStyleSheet(R"(
                                    border-radius: 50%;
                                    color: rgb(255, 255, 255);
                                    border-image: url(:/resource/sound/音量.svg);)");
    }

    YAML::Node task_node;
    is_sound = !is_sound;
    ui_page1->setBackgroundSoudFalg(is_sound);
    task_node["brackground_sound"] = is_sound;
    std::ofstream fout(qt_node_->config_path + "/config_sound.yaml");
    fout << task_node;
    fout.close();
}

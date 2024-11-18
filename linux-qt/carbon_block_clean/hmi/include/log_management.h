#ifndef LOG_MANAGEMENT_H
#define LOG_MANAGEMENT_H

#include <QWidget>
#include <QDateTime>
#include <QTime>
#include "page_widget.h"

namespace Ui {
class Log_management;
}

class Log_management : public QWidget
{
    Q_OBJECT

public:
    explicit Log_management(QWidget *parent = nullptr);
    ~Log_management();

private:
    Ui::Log_management *ui;
    Page_widget *pageWidget3;
    int log_page_index = 1; // 当前日志页数
    int log_row_num = 10;   // 每页显示的行数
    int log_column_num = 3; // 每页显示的列数
    int log_page_sum = 1;   // 日志总页数
    int log_item_sum;            // 日志总条数
    int log_last_page_item = 0;  // 最后一页日志条数
    void init_log_tableWidget();
    QString load_log_log(int i); // 日志文件输入处
    int m_currentPage; //当前页面
    void setLogPage();
    void showLogData(int page);

private slots:
    void LogPageChanged(int page);
};

#endif // LOG_MANAGEMENT_H

#ifndef REPORT_CHECK_H
#define REPORT_CHECK_H

#include <QWidget>
#include <QDateTime>
#include <QTime>
#include "page_widget.h"

namespace Ui {
class Report_check;
}

class Report_check : public QWidget
{
    Q_OBJECT

public:
    explicit Report_check(QWidget *parent = nullptr);
    ~Report_check();

private:
    Ui::Report_check *ui;
    Page_widget *pageWidget2;
    int report_page_index = 1; // 当前日志页数
    int report_row_num = 10;   // 每页显示的行数
    int report_column_num = 7; // 每页显示的列数
    int report_page_sum = 1;   // 日志总页数
    int report_item_sum;            // 日志总条数
    int report_last_page_item = 0;  // 最后一页日志条数
    void init_report_tableWidget();
    QString load_log_report(int i); // 日志文件输入处
    int m_currentPage; //当前页面
    void setReportPage();
    void showReportData(int page);

private slots:
    void ReportPageChanged(int page);
};

#endif // REPORT_CHECK_H

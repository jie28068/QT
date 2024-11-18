#ifndef DATA_CHECK_H
#define DATA_CHECK_H

#include <QWidget>
#include <QDateTime>
#include <QTime>
#include "page_widget.h"

namespace Ui {
class Data_check;
}

class Data_check : public QWidget
{
    Q_OBJECT

public:
    explicit Data_check(QWidget *parent = nullptr);
    ~Data_check();

private:
    Ui::Data_check *ui;
    Page_widget *pageWidget;
    int data_page_index = 1; // 当前日志页数
    int data_row_num = 10;   // 每页显示的行数
    int data_column_num = 6; // 每页显示的列数
    int data_page_sum = 1;   // 日志总页数
    int data_item_sum;            // 日志总条数
    int data_last_page_item = 0;  // 最后一页日志条数
    void init_data_tableWidget();
    QString load_log_data(int i); // 日志文件输入处
    int m_currentPage; //当前页面
    void setDataPage();
    void showDataPage(int page);

private slots:
    void DataPageChanged(int page);
};

#endif // DATA_CHECK_H

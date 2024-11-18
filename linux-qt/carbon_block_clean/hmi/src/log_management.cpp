#include "log_management.h"
#include "ui_log_management.h"

Log_management::Log_management(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Log_management)
{
    ui->setupUi(this);
    // 设置QDateTimeEdit下拉框日历模式
    ui->dateTimeEdit_start->setCalendarPopup(true);
    ui->dateTimeEdit_end->setCalendarPopup(true);
    // 设置QDateTimeEdit的日期
    ui->dateTimeEdit_start->setDate(QDate::currentDate());
    ui->dateTimeEdit_end->setDate(QDate::currentDate());

    init_log_tableWidget();
    pageWidget3 = new Page_widget;
    connect(pageWidget3, &Page_widget::currentPageChanged, this, &Log_management::LogPageChanged);

    ui->verticalLayout3->addWidget(pageWidget3);
    ui->verticalLayout3->setSizeConstraint(QLayout::SetFixedSize);

    m_currentPage = 1;
    setLogPage();
}

Log_management::~Log_management()
{
    delete ui;
}

void Log_management::init_log_tableWidget() {
    // 设置表格样式，不显示网格线
    ui->tableWidget->setShowGrid(false);
    // 设置列宽
    ui->tableWidget->setColumnWidth(0, 400);
    ui->tableWidget->setColumnWidth(1, 600);
    ui->tableWidget->setColumnWidth(2, 400);
    // 设置行高
    for (int i = 0; i < log_row_num; i++){
        ui->tableWidget->setRowHeight(i, 50);
    }
}

QString Log_management::load_log_log(int i)//设置显示内容 以下为模板文字 日志在这里输入界面
{
    QString string = "";
    if(i==0){
        string = "2024-08-29 18:47";
    }
    if (i == 1) {
        string = "负压低故障";
    }
    if(i==2){
        string = "error";
    }
    return string;
}

void Log_management::LogPageChanged(int page) {
    showLogData(page);
}

void Log_management::setLogPage() {
    double value = 300.0/10;
    int ceil_value = qCeil(value);
    pageWidget3->setMaxPage(ceil_value);
    showLogData(1);
}

void Log_management::showLogData(int page) {
    m_currentPage = page;

//    int records = (page*10 > 200) ? (200-(page-1)*10) : 10;
//        ui->tableWidget->setRowCount(0);
//        int start = (page-1)*10;
//        for (int i = 0; i < records; i++)
//        {
//            int rowCount = ui->tableWidget->rowCount();
//            ui->tableWidget->insertRow(rowCount);

//            ui->tableWidget->setItem(i, 0, new QTableWidgetItem(QString("%1").arg(start+i+1)));
//            ui->tableWidget->setItem(i, 1, new QTableWidgetItem("1"));
//        }
    // 设置模板内容
    for (int i = 0; i < log_row_num; i++){
        for (int j = 0; j < log_column_num; j++){
            QString string = "";
            string = load_log_log(j);
            QTableWidgetItem *item = new QTableWidgetItem(string);
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & (~Qt::ItemIsEditable));    	// 设置可选不可改
            ui->tableWidget->setItem(i, j, item);
        }
    }
}

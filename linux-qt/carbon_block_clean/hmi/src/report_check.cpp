#include "report_check.h"
#include "ui_report_check.h"

Report_check::Report_check(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Report_check)
{
    ui->setupUi(this);
    // 设置QDateTimeEdit下拉框日历模式
    ui->dateTimeEdit_start->setCalendarPopup(true);
    ui->dateTimeEdit_end->setCalendarPopup(true);
    // 设置QDateTimeEdit的日期
    ui->dateTimeEdit_start->setDate(QDate::currentDate());
    ui->dateTimeEdit_end->setDate(QDate::currentDate());

    init_report_tableWidget();
    pageWidget2 = new Page_widget;
    connect(pageWidget2, &Page_widget::currentPageChanged, this, &Report_check::ReportPageChanged);

    ui->verticalLayout2->addWidget(pageWidget2);
    ui->verticalLayout2->setSizeConstraint(QLayout::SetFixedSize);

    m_currentPage = 1;
    setReportPage();
}

Report_check::~Report_check()
{
    delete ui;
}

void Report_check::init_report_tableWidget() {
    // 设置表格样式，不显示网格线
    ui->tableWidget->setShowGrid(false);
    // 设置列宽
    ui->tableWidget->setColumnWidth(0, 200);
    ui->tableWidget->setColumnWidth(1, 200);
    ui->tableWidget->setColumnWidth(2, 200);
    ui->tableWidget->setColumnWidth(3, 200);
    ui->tableWidget->setColumnWidth(4, 200);
    ui->tableWidget->setColumnWidth(5, 200);
    ui->tableWidget->setColumnWidth(6, 280);
    // 设置行高
    for (int i = 0; i < report_row_num; i++){
        ui->tableWidget->setRowHeight(i, 50);
    }
}

QString Report_check::load_log_report(int i)//设置显示内容 以下为模板文字 日志在这里输入界面
{
    QString string = "";
    if(i==0){
        string = "2024-08-29 17:07";
    }
    if (i == 1) {
        string = "1#";
    }
    if(i==2){
        string = "100";
    }
    if (i == 3) {
        string = "89";
    }
    if (i == 4) {
        string = "20";
    }
    if (i == 5) {
        string = "50min";
    }
    if (i == 6) {
        string = "优";
    }
    return string;
}

void Report_check::ReportPageChanged(int page) {
    showReportData(page);
}

void Report_check::setReportPage() {
    double value = 200.0/10;
    int ceil_value = qCeil(value);
    pageWidget2->setMaxPage(ceil_value);
    showReportData(1);
}

void Report_check::showReportData(int page) {
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
    for (int i = 0; i < report_row_num; i++){
        for (int j = 0; j < report_column_num; j++){
            QString string = "";
            string = load_log_report(j);
            QTableWidgetItem *item = new QTableWidgetItem(string);
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & (~Qt::ItemIsEditable));    	// 设置可选不可改
            ui->tableWidget->setItem(i, j, item);
        }
    }
}

#include "data_check.h"
#include "ui_data_check.h"

Data_check::Data_check(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Data_check)
{
    ui->setupUi(this);
    // 设置QDateTimeEdit下拉框日历模式
    ui->dateTimeEdit_start->setCalendarPopup(true);
    ui->dateTimeEdit_end->setCalendarPopup(true);
    // 设置QDateTimeEdit的日期
    ui->dateTimeEdit_start->setDate(QDate::currentDate());
    ui->dateTimeEdit_end->setDate(QDate::currentDate());

    init_data_tableWidget();
    pageWidget = new Page_widget;
    connect(pageWidget, &Page_widget::currentPageChanged, this, &Data_check::DataPageChanged);

    ui->verticalLayout->addWidget(pageWidget);
    ui->verticalLayout->setSizeConstraint(QLayout::SetFixedSize);

    m_currentPage = 1;
    setDataPage();

}

Data_check::~Data_check()
{
    delete ui;
}

void Data_check::init_data_tableWidget() {
    // 设置表格样式，不显示网格线
    ui->tableWidget->setShowGrid(false);
    // 设置列宽
    ui->tableWidget->setColumnWidth(0, 200);
    ui->tableWidget->setColumnWidth(1, 100);
    ui->tableWidget->setColumnWidth(2, 300);
    ui->tableWidget->setColumnWidth(3, 200);
    ui->tableWidget->setColumnWidth(4, 200);
    ui->tableWidget->setColumnWidth(5, 500);
    // 设置行高
    for (int i = 0; i < data_row_num; i++){
        ui->tableWidget->setRowHeight(i, 50);
    }
}

QString Data_check::load_log_data(int i)//设置显示内容 以下为模板文字 日志在这里输入界面
{
    QString string = "";
    if(i==0){
        string = "K12";
    }
    if (i == 1) {
        string = "1#";
    }
    if(i==2){
        string = "2024-08-29 09:38";
    }
    if (i == 3) {
        string = "是";
    }
    if (i == 4) {
        string = "20min";
    }
    if (i == 5) {
        string = "无操作";
    }
    return string;
}

void Data_check::DataPageChanged(int page) {
    showDataPage(page);
}

void Data_check::setDataPage() {
    double value = 200.0/10;
    int ceil_value = qCeil(value);
    pageWidget->setMaxPage(ceil_value);
    showDataPage(1);
}

void Data_check::showDataPage(int page) {
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
    for (int i = 0; i < data_row_num; i++){
        for (int j = 0; j < data_column_num; j++){
            QString string = "";
            string = load_log_data(j);
            QTableWidgetItem *item = new QTableWidgetItem(string);
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & (~Qt::ItemIsEditable));    	// 设置可选不可改
            ui->tableWidget->setItem(i, j, item);
        }
    }
}

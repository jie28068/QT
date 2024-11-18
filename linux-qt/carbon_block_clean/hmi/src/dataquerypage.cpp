#include "dataquerypage.h"
#include "ui_dataquerypage.h"
#include "qtablepages.h"

DataQueryPage::DataQueryPage(QWidget *parent) : QWidget(parent),
                                                ui(new Ui::DataQueryPage)
{
    ui->setupUi(this);
    // 设置QDateTimeEdit下拉框日历模式
    ui->dateTimeEdit_start->setCalendarPopup(true);
    ui->dateTimeEdit_end->setCalendarPopup(true);
    // 设置QDateTimeEdit的日期
    ui->dateTimeEdit_start->setDate(QDate::currentDate());
    ui->dateTimeEdit_end->setDate(QDate::currentDate());

    QStringList headers;
    headers << QString("序号") << QString("用户名") << QString("用户ID") << QString("时间戳") << QString("设备") << QString("事件");

    /* 初始化数据填充 模拟 */
    QList<QStringList> sampleList;
    for (int i = 0; i < 20229; i++)
    {
        QStringList tempData;
        for (int col = 0; col < headers.size(); col++)
        {
            if (col == 0)
            {
                tempData.append(QString::fromLocal8Bit("%1").arg(i + 1)); // 设置序号
            }
            else
            {
                tempData.append(QString::fromLocal8Bit("L%1_columns%2").arg(i + 1).arg(col + 1)); // 表格内容
            }
        }
        sampleList.append(tempData);
    }

    QTablePages *tab = new QTablePages();
    ui->widget_tab->InitTableForm(headers, sampleList, 20);
    connect(ui->lineEdit, &QLineEdit::textChanged, [=](const QString &str)
            { tab->SearchTableData(str); });
}

DataQueryPage::~DataQueryPage()
{
    delete ui;
}

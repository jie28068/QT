#include "dataTableWidget.h"
#include "qtnode.h"
#include "globals.h"

#include <QFile>
#include <QHBoxLayout>
#include <QVBoxLayout>
DataTableWidget::DataTableWidget(QtNode *node_ptr, QString str, QWidget *parent) : QWidget(parent), qt_node_(node_ptr), logType(str)
{
    setupUI();
    connectSignalsSlots();
    loadData();

    // 加载样式
    QFile files(":/qss/dataTableWidget.qss");
    if (files.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        setStyleSheet(files.readAll());
        files.close();
    }

    pageSizeLabel->setText(QString("每页行数[%1]").arg(pageSize));
    allDataLabel->setText(QString("共 [%1] 行数据").arg(totalRows));
}

void DataTableWidget::loadData()
{

    QVector<QVector<QString>> dataSource;
    if (logType == globals::systemLog)
    {
        dataSource = qt_node_->get_system_logs();
    }
    else if (logType == globals::operationLog)
    {
        dataSource = qt_node_->get_operation_logs();
    }

    // 清除旧数据
    tableWidget->clearContents();
    tableWidget->setRowCount(0);
    // 设置数据源
    this->dataSource = dataSource;
    // 设置每页显示的行数
    if (globals::operationLog == logType)
    {
        pageSize = qt_node_->m_operation_log_row_count;
    }
    else if (logType == globals::systemLog)
    {
        pageSize = qt_node_->m_page_row_count;
    }
    // 设置总数据行数
    totalRows = dataSource.size();
    // 计算总页数
    totalPage = (totalRows + pageSize - 1) / pageSize;

    updateRowPageLabel();
}

void DataTableWidget::displayPage(int page)
{
    // 清除旧数据
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    // 计算当前页的起始和结束行号
    int startRow = (page - 1) * pageSize;
    int endRow = qMin(startRow + pageSize, totalRows);

    // 添加新数据到表格
    for (int i = startRow; i < endRow; ++i)
    {
        int row = tableWidget->rowCount();
        tableWidget->insertRow(row);
        for (int col = 0; col < tableWidget->columnCount(); ++col)
        {
            QTableWidgetItem *item = new QTableWidgetItem(dataSource[i][col]);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable & ~Qt::ItemIsSelectable);
            if (col != tableWidget->columnCount() - 1) // 设置除最后一列内容都居中
            {
                item->setTextAlignment(Qt::AlignCenter);
            }

            // 设置颜色
            if (dataSource[i][0] == globals::normalMessage)
            {
                item->setForeground(Qt::white);
            }
            else if (dataSource[i][0] == globals::warningMessage)
            {
                item->setForeground(Qt::yellow);
            }
            else if (dataSource[i][0] == globals::errorMessage)
            {
                item->setForeground(Qt::red);
            }

            tableWidget->setItem(row, col, item);
        }
    }

    // 更新页码标签
    currentPage = page;
    updatePageLabel();
}

void DataTableWidget::onPreviousPage()
{
    if (currentPage > 1)
    {
        displayPage(currentPage - 1);
    }
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void DataTableWidget::onNextPage()
{
    if (currentPage < totalPage)
    {
        displayPage(currentPage + 1);
        tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    }
    if (currentPage == totalPage) // 最后一页，设置固定高度
        tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void DataTableWidget::onPreviousRowPage()
{
    if (pageSize > 5)
    {
        pageSize = pageSize - 1;
        updateRowPageLabel();
    }
}
void DataTableWidget::onNextRowPage()
{
    if (pageSize < totalRows)
    {
        pageSize = pageSize + 1;
        updateRowPageLabel();
    }
}

void DataTableWidget::onRefresh()
{
    loadData();
}

void DataTableWidget::setupUI()
{
    // 设置垂直布局
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // 创建表格
    tableWidget = new QTableWidget(this);
    tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    if (globals::operationLog == logType)
    {
        tableWidget->setColumnCount(2);
        tableWidget->setHorizontalHeaderLabels(QStringList() << "时间"
                                                             << "内容");
        QHeaderView *header = tableWidget->horizontalHeader();
        header->setSectionResizeMode(0, QHeaderView::Interactive);
        tableWidget->setColumnWidth(0, 300);
        header->setSectionResizeMode(1, QHeaderView::Stretch);
    }
    else
    {
        tableWidget->setColumnCount(3);
        tableWidget->setHorizontalHeaderLabels(QStringList() << "级别"
                                                             << "时间"
                                                             << "内容");
        QHeaderView *header = tableWidget->horizontalHeader();
        header->setSectionResizeMode(0, QHeaderView::Interactive);
        header->setSectionResizeMode(1, QHeaderView::Interactive);
        tableWidget->setColumnWidth(0, 100);
        tableWidget->setColumnWidth(1, 300);
        header->setSectionResizeMode(2, QHeaderView::Stretch);
    }

    // 设置表格自适应布局
    // tableWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    // tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    // 添加到布局中
    mainLayout->addWidget(tableWidget);

    // 创建翻页按钮、标签和页大小选择框
    QHBoxLayout *buttonLayout = new QHBoxLayout(this);
    previousButton = new QPushButton(this);
    previousButton->setObjectName("previousButton");
    nextButton = new QPushButton(this);
    nextButton->setObjectName("nextButton");
    pageLabel = new QLabel(this);
    pageSizeLabel = new QLabel(this);
    refreshButton = new QPushButton("刷新", this);
    refreshButton->setObjectName("refreshButton");

    previousPageButton = new QPushButton(this);
    previousPageButton->setObjectName("previousPageButton");
    nextPageButton = new QPushButton(this);
    nextPageButton->setObjectName("nextPageButton");

    previousButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    nextButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    nextPageButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    previousPageButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    refreshButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    previousPageButton->setMaximumSize(80, 100);
    nextPageButton->setMaximumSize(80, 100);
    previousButton->setMaximumSize(80, 100);
    nextButton->setMaximumSize(80, 100);
    refreshButton->setMaximumSize(80, 50);

    allDataLabel = new QLabel(this);
    buttonLayout->addWidget(allDataLabel);
    buttonLayout->addSpacerItem(new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));
    QHBoxLayout *layoutSipn = new QHBoxLayout(this);
    QHBoxLayout *layoutBtn = new QHBoxLayout(this);
    layoutBtn->addWidget(previousButton);
    layoutBtn->addWidget(pageLabel);
    layoutBtn->addWidget(nextButton);
    buttonLayout->addLayout(layoutSipn);
    buttonLayout->setContentsMargins(50, 20, 0, 20);
    buttonLayout->addSpacerItem(new QSpacerItem(50, 20, QSizePolicy::Maximum, QSizePolicy::Minimum));

    layoutSipn->addWidget(previousPageButton);
    layoutSipn->addWidget(pageSizeLabel);
    layoutSipn->addWidget(nextPageButton);

    buttonLayout->addLayout(layoutBtn);
    buttonLayout->addSpacerItem(new QSpacerItem(50, 20, QSizePolicy::Maximum, QSizePolicy::Minimum));

    buttonLayout->addWidget(refreshButton);
    buttonLayout->addSpacerItem(new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum));

    mainLayout->addLayout(buttonLayout);

    // mainLayout->setStretchFactor(tableWidget, 8);
    // mainLayout->setStretchFactor(buttonLayout, 1);
    // 设置窗口布局
    setLayout(mainLayout);
}

void DataTableWidget::connectSignalsSlots()
{
    connect(previousButton, &QPushButton::clicked, this, &DataTableWidget::onPreviousPage);
    connect(nextButton, &QPushButton::clicked, this, &DataTableWidget::onNextPage);
    connect(previousPageButton, &QPushButton::clicked, this, &DataTableWidget::onPreviousRowPage);
    connect(nextPageButton, &QPushButton::clicked, this, &DataTableWidget::onNextRowPage);
    connect(refreshButton, &QPushButton::clicked, this, &DataTableWidget::onRefresh);
}

void DataTableWidget::updatePageLabel()
{
    pageLabel->setText(QString("页码 %1/%2").arg(currentPage).arg(totalPage));
}

void DataTableWidget::updateRowPageLabel()
{
    displayPage(1);
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    totalPage = (totalRows + pageSize - 1) / pageSize;
    if (globals::operationLog == logType)
    {
        qt_node_->m_operation_log_row_count = pageSize;
    }
    else if (logType == globals::systemLog)
    {
        qt_node_->m_page_row_count = pageSize;
    }
    pageSizeLabel->setText(QString("每页行数[%1]").arg(pageSize));
    allDataLabel->setText(QString("共 [%1] 行数据").arg(totalRows));
    qt_node_->save_config_info(qt_node_->config_path + "config_task.yaml");
    updatePageLabel();
}
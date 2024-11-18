#include "page_widget.h"
#include "ui_page_widget.h"

Page_widget::Page_widget(int blockSize, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Page_widget)
{
    ui->setupUi(this);
    initPageWidget(blockSize);
}

Page_widget::~Page_widget()
{
    delete ui;
    delete pageLabels;
}

void Page_widget::initPageWidget(int blockSize) {
    font = QFont("Times New Roman", 14);
    font.setBold(true);
    // 前一页， "<"
    previousPageLabel = new QLabel;
    previousPageLabel->setFont(font);
    previousPageLabel->setAlignment(Qt::AlignCenter);
    previousPageLabel->setFixedSize(23, 23);
    previousPageLabel->setText("<");
    previousPageLabel->setStyleSheet("QLabel{color:rgba(255,255,255,0.85); padding:2px;}"
                                     "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");
    // 左侧部分标签的容器
    leftPagesWidget = new QWidget;
    leftPagesWidget->resize(23,23);
    leftPagesWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    // 左侧分隔符， ".."
    leftSeparateLabel = new QLabel;
    leftSeparateLabel->setFont(font);
    leftSeparateLabel->setAlignment(Qt::AlignCenter);
    leftSeparateLabel->setFixedSize(23,23);
    leftSeparateLabel->setText("..");
    leftSeparateLabel->setStyleSheet("QLabel{color:rgba(255,255,255,0.85);}");
    // 中间部分标签的容器
    centerPagesWidget = new QWidget;
    centerPagesWidget->resize(23,23);
    centerPagesWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    // 右侧分隔符， ".."
    rightSeparateLabel = new QLabel;
    rightSeparateLabel->setFont(font);
    rightSeparateLabel->setAlignment(Qt::AlignCenter);
    rightSeparateLabel->setFixedSize(23,23);
    rightSeparateLabel->setText("..");
    rightSeparateLabel->setStyleSheet("QLabel{color:rgba(255,255,255,0.85);}");
    // 右侧部分标签的容器
    rightPagesWidget = new QWidget;
    rightPagesWidget->resize(23,23);
    rightPagesWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    // 下一页，">"
    nextPageLabel = new QLabel;
    nextPageLabel->setFont(font);
    nextPageLabel->setAlignment(Qt::AlignCenter);
    nextPageLabel->setFixedSize(23,23);
    nextPageLabel->setText(">");
    nextPageLabel->setStyleSheet("QLabel{color:rgba(255,255,255,0.85); padding:2px;}"
                                "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");

    QHBoxLayout * mainLayout = new QHBoxLayout;
    mainLayout->setMargin(0);

    mainLayout->addWidget(ui->label_text_sum_items);
    mainLayout->insertSpacing(1, 80);
    mainLayout->addWidget(previousPageLabel);
    mainLayout->addWidget(leftPagesWidget);
    mainLayout->addWidget(leftSeparateLabel);
    mainLayout->addWidget(centerPagesWidget);
    mainLayout->addWidget(rightSeparateLabel);
    mainLayout->addWidget(rightPagesWidget);
    mainLayout->addWidget(nextPageLabel);
    mainLayout->insertSpacing(9, 900);
    mainLayout->addWidget(ui->label_goto);
    ui->pageLineEdit->setFixedWidth(30);
    mainLayout->addWidget(ui->pageLineEdit);
    mainLayout->addWidget(ui->label_page);
    setLayout(mainLayout);

    setBlockSize(blockSize);

    initialize();

    maxPage = 0;
    setMaxPage(1);
    ui->pageLineEdit->setText(QString::number(1));
}

// 获取每部分的标签个数
int Page_widget::getBlockSize() const
{
    return blockSize;
}

// 设置每部分的标签个数，为了便于计算, block size 必须是奇数, 且最小为3
void Page_widget::setBlockSize(int blockSize)
{
    blockSize = qMax(blockSize, 3);
    if(blockSize%2 == 0)
    {
        ++blockSize;
    }
    this->blockSize = blockSize;
}

// 获取总页数
int Page_widget::getMaxPage() const
{
    return maxPage;
}

// 设置总页数
void Page_widget::setMaxPage(int page)
{
    page = qMax(page, 1);
    if(maxPage != page)
    {
        this->maxPage = page;
        this->currentPage = 1;
        updatePageLabels();
    }
}

// 获取当前页数
int Page_widget::getCurrentPage() const
{
    return currentPage;
}

// 设置当前页
void Page_widget::setCurrentPage(int page, bool signalEmitted)
{
    page = qMax(page, 1);
    page = qMin(page, maxPage);

    if(page != this->currentPage)
    {
        this->currentPage = page;
        updatePageLabels();

        ui->pageLineEdit->setText(QString::number(page));
        if(signalEmitted)
        {
            emit currentPageChanged(page);
        }
    }
}

// 事件过滤器，响应上一页标签和下一页标签的点击事件
bool Page_widget::eventFilter(QObject * watched, QEvent * e)
{
    if(e->type() == QEvent::MouseButtonRelease)
    {
        int page = -1;

        // 点击了前一页标签
        if(watched == previousPageLabel)
        {
            page = getCurrentPage()-1;
        }

        // 点击了后一页标签
        if(watched == nextPageLabel)
        {
            page = getCurrentPage()+1;
        }

        // 点击了具体数字的标签
        for(int i=0; i<pageLabels->count(); i++)
        {
            if(watched == pageLabels->at(i))
            {
                page = pageLabels->at(i)->text().toInt();
                break;
            }
        }

        if(page != -1)
        {
            setCurrentPage(page, true);
            return true;
        }
    }

    if (watched == ui->pageLineEdit && e->type() == QEvent::KeyRelease) {
        QKeyEvent *ke = static_cast<QKeyEvent *>(e);
        if (ke->key() == Qt::Key_Enter || ke->key() == Qt::Key_Return)
        {
            int page = ui->pageLineEdit->text().toInt();
            if(page > maxPage)
            {
                page = maxPage;
                ui->pageLineEdit->setText(QString::number(page));
            }
            setCurrentPage(page, true);
            return true;
        }
    }

    return QWidget::eventFilter(watched, e);
}

// 页码标签初始化，分成三个部分, 左...中...右
void Page_widget::initialize()
{
    ui->pageLineEdit->installEventFilter(this);
    ui->pageLineEdit->setValidator(new QIntValidator(1, 10000000, this));

    previousPageLabel->setProperty("page", "true");
    nextPageLabel->setProperty("page", "true");
    previousPageLabel->installEventFilter(this);
    nextPageLabel->installEventFilter(this);

    pageLabels = new QList<QLabel *>();

    QHBoxLayout * leftLayout = new QHBoxLayout();
    leftLayout->setMargin(0);
    leftLayout->setContentsMargins(0,0,0,0);
    leftLayout->setSpacing(0);

    QHBoxLayout * centerLayout = new QHBoxLayout();
    centerLayout->setMargin(0);
    centerLayout->setContentsMargins(0,0,0,0);
    centerLayout->setSpacing(0);

    QHBoxLayout * rightLayout = new QHBoxLayout();
    rightLayout->setMargin(0);
    rightLayout->setContentsMargins(0,0,0,0);
    rightLayout->setSpacing(0);

    for(int i=0; i<blockSize*3; ++i)
    {
        QLabel * label = new QLabel(QString::number(i+1));
        font.setFamily("Times New Roman");
        label->setFont(font);
        label->setProperty("page", "true");
        label->setAlignment(Qt::AlignCenter);
        label->setFixedHeight(23);
        label->setMinimumWidth(23);
        label->setText(QString::number(i+1));
        label->setStyleSheet("QLabel{color:rgba(255,255,255,0.85); padding:2px;}"
                             "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");
        label->installEventFilter(this);

        pageLabels->append(label);

        if(i<blockSize)
        {
            leftLayout->addWidget(label);
        }
        else if(i<blockSize*2)
        {
            centerLayout->addWidget(label);
        }
        else
        {
            rightLayout->addWidget(label);
        }
    }

    leftPagesWidget->setLayout(leftLayout);
    centerPagesWidget->setLayout(centerLayout);
    rightPagesWidget->setLayout(rightLayout);
}

// 更新显示标签
void Page_widget::updatePageLabels()
{
    leftSeparateLabel->hide();
    rightSeparateLabel->hide();

    // 总页数小于 blockSize*3，总页数数值之前的 Label 都显示，之后的都隐藏
    if(maxPage <= blockSize*3)
    {
        for(int i=0; i<pageLabels->count(); i++)
        {
            QLabel * label = pageLabels->at(i);
            if(i<maxPage)
            {
                label->setText(QString::number(i+1));
                label->show();
            }
            else
            {
                label->hide();
            }

            if(currentPage-1 == i)
            {
                label->setProperty("currentPage", "true");
                // 当前页的字体设置为蓝色
                label->setStyleSheet("QLabel{color:rgba(255,0,0,0.85); padding:2px;}"
                                     "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");
            }
            else
            {
                label->setProperty("currentPage", "false");
                // 非当前页的字体设置为白色
                label->setStyleSheet("QLabel{color:rgba(255,255,255,0.85); padding:2px;}"
                                     "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");
            }
            label->setStyleSheet("/**/");
        }
        return;
    }

    // 以下情况为 maxPage 大于blockSize * 3, 所有的页码label都要显示
    // c 为 currentPage
    // n 为 block size
    // m 为 maxPage

    // 1. c ∈ [1, n + n/2 + 1]: 显示前 n * 2 个, 后 n 个: 只显示右边的分隔符
    // 2. c ∈ [m - n - n/2, m]: 显示前 n 个, 后 n * 2 个: 只显示左边的分隔符
    // 3. 显示[1, n], [c - n/2, c + n/2], [m - 2*n + 1, m]: 两个分隔符都显示

    int c = currentPage;
    int n = blockSize;
    int m = maxPage;
    int centerStartPage = 0;

    if(c >= 1 && c <= n+n/2+1)
    {
        // 1. c ∈ [1, n + n/2 + 1]: 显示前 n * 2 个, 后 n 个: 只显示右边的分隔符
        centerStartPage = n+1;
        rightSeparateLabel->show();
    }
    else if(c >= m-n-n/2 && c <= m)
    {
        // 2. c ∈ [m - n - n/2, m]: 显示前 n 个, 后 n * 2 个: 只显示左边的分隔符
        centerStartPage = m-n-n+1;
        leftSeparateLabel->show();
    }
    else
    {
        // 3. 显示[1, n], [c - n/2, c + n/2], [m - n + 1, m]: 两个分隔符都显示
        centerStartPage = c-n/2;
        rightSeparateLabel->show();
        leftSeparateLabel->show();
    }

    for(int i=0; i<n; ++i)
    {
        pageLabels->at(i)->setText(QString::number(i+1));                        // 前面 n 个
        pageLabels->at(n+i)->setText(QString::number(centerStartPage+i));        // 中间 n 个
        pageLabels->at(3*n-i-1)->setText(QString::number(m-i));                  // 后面 n 个
    }

    for(int i=0; i<pageLabels->count(); ++i)
    {
        QLabel * label = pageLabels->at(i);
        int page = label->text().toInt();
        if(page == currentPage)
        {
            // 当前页的字体设置为蓝色
            label->setStyleSheet("QLabel{color:rgba(255,0,0,0.85); padding:2px;}"
                                 "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");
        }
        else
        {
            // 非当前页的字体设置为白色
            label->setStyleSheet("QLabel{color:rgba(255,255,255,0.85); padding:2px;}"
                                 "QLabel:hover{color: black; border-radius: 4px; background-color: qlineargradient(spread:reflect, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(53, 121, 238, 255), stop:1 rgba(0, 202, 237, 255));}");

        }
        label->show();
    }
}

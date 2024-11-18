#include "pagenavigator.h"
#include "ui_pagenavigator.h"
#include <QDebug>

PageNavigator::PageNavigator(QWidget *parent) : QWidget(parent),
                                                ui(new Ui::PageNavigator)
{
    ui->setupUi(this);

    m_maxPage = 1;
    m_curPage = 1;

    QStringList strList;
    strList << "10"
            << "20"
            << "50"
            << "100"
            << "200"
            << "500"; // set fix enum
    ui->pagePerCount->addItems(strList);

    ui->tBtnSwitch->setProperty("state", "enable");

    UpdateButtonState();
    connect(parent, SIGNAL(SigSearchResults(QList<QStringList>)), this, SLOT(OnSearchResults(QList<QStringList>)));
}

PageNavigator::~PageNavigator()
{
    delete ui;
}

void PageNavigator::InitNavbar(int total, int pageSize)
{
    if (total < 0 || pageSize < 0)
    {
        return;
    }

    m_totalRecord = total;
    ui->labelTotalRecord->setText(QString("共有%1条记录").arg(m_totalRecord));
    ui->lineEditPage->setValidator(new QIntValidator(1, 1000000, this));

    // 适配pageSize参数, 每页显示行数 pagePerCount 采用固定枚举值
    int index = 0;
    int selectCount = ui->pagePerCount->count();

    if (pageSize < ui->pagePerCount->itemText(0).toInt())
    {
        index = 0;
    }
    if (pageSize >= ui->pagePerCount->itemText(selectCount - 1).toInt())
    {
        index = selectCount - 1;
    }
    for (int i = 0; i < ui->pagePerCount->count(); i++)
    {
        if (pageSize >= ui->pagePerCount->itemText(i).toInt() &&
            pageSize < ui->pagePerCount->itemText(i + 1).toInt())
        {
            index = i;
        }
    }

    ui->pagePerCount->setCurrentIndex(index);

    // 初始化显示内容
    if (m_totalRecord > 0)
    {
        m_pageLines = ui->pagePerCount->itemText(index).toInt();
        m_maxPage = (m_totalRecord + (m_pageLines - 1)) / m_pageLines;
        ui->labelCurPage->setText(QString("第%1页/共%2页").arg(m_curPage).arg(m_maxPage));
        ui->lineEditPage->clear();

        // 显示首页
        m_curPage = 0;
        SetCurrentPage(1);
    }
}

void PageNavigator::ResetNavBar()
{
    m_totalRecord = 0;
    ui->labelTotalRecord->setText(QString("共有%1条记录").arg(m_totalRecord));

    ui->pagePerCount->setCurrentIndex(0);
    m_pageLines = ui->pagePerCount->itemText(0).toInt();

    ui->lineEditPage->clear();

    m_maxPage = 1;
    m_curPage = 0;
    SetCurrentPage(1);
}

void PageNavigator::SetCurrentPage(int page)
{
    if (m_curPage == page)
    {
        return;
    }

    if (page < 1)
    {
        page = 1;
    }

    if (page > (int)m_maxPage)
    {
        page = m_maxPage;
    }

    qDebug() << "page :" << page << "m_maxPage :" << m_maxPage;

    m_curPage = page;
    ui->labelCurPage->setText(QString("第%1页/共%2页").arg(m_curPage).arg(m_maxPage));

    UpdateButtonState();

    emit SigCurrentPageChanged(m_curPage); // 通知 table 刷新

    // // 当页码有效时，更新当前页并发出信号
    // if (page >= 1 && page <= m_maxPage)
    // {
    //     m_curPage = page;
    //     ui->labelCurPage->setText(QString("第%1页/共%2页").arg(m_curPage).arg(m_maxPage));

    //     UpdateButtonState();

    //     emit SigCurrentPageChanged(m_curPage); // 通知 table 刷新
    // }
}

void PageNavigator::UpdatePages()
{
}

void PageNavigator::HandlePageLinesChanged(int count)
{
    if (m_totalRecord == 0)
    {
        return;
    }

    // 与当前页一致
    if (m_pageLines == count)
    {
        return;
    }

    qDebug() << "HandlePageLinesChanged reset page count:" << m_pageLines << "-->" << count;

    m_pageLines = count;
    m_maxPage = (m_totalRecord + (m_pageLines - 1)) / m_pageLines;
    ui->labelCurPage->setText(QString("第%1页/共%2页").arg(m_curPage).arg(m_maxPage));
    ui->lineEditPage->clear();

    // 回到首页
    m_curPage = 0;
    SetCurrentPage(1);
}

// 设置按钮状态: Add setProperty for QSS
void PageNavigator::UpdateButtonState()
{
    if (m_curPage == 1 && m_maxPage == 1)
    {
        // 单页显示 single page
        ui->tBtnFirst->setEnabled(false);
        ui->tBtnFirst->setProperty("state", "disable");
        ui->tBtnLast->setEnabled(false);
        ui->tBtnLast->setProperty("state", "disable");
        ui->tBtnPrevious->setEnabled(false);
        ui->tBtnPrevious->setProperty("state", "disable");
        ui->tBtnNext->setEnabled(false);
        ui->tBtnNext->setProperty("state", "disable");
    }
    else if (m_curPage == 1 && m_maxPage > 1)
    {
        // 首页显示 first page
        ui->tBtnFirst->setEnabled(false);
        ui->tBtnFirst->setProperty("state", "disable");
        ui->tBtnLast->setEnabled(true);
        ui->tBtnLast->setProperty("state", "enable");
        ui->tBtnPrevious->setEnabled(false);
        ui->tBtnPrevious->setProperty("state", "disable");
        ui->tBtnNext->setEnabled(true);
        ui->tBtnNext->setProperty("state", "enable");
    }
    else if (m_curPage == m_maxPage)
    {
        // 尾页显示 last page
        ui->tBtnFirst->setEnabled(true);
        ui->tBtnFirst->setProperty("state", "enable");
        ui->tBtnLast->setEnabled(false);
        ui->tBtnLast->setProperty("state", "disable");
        ui->tBtnPrevious->setEnabled(true);
        ui->tBtnPrevious->setProperty("state", "enable");
        ui->tBtnNext->setEnabled(false);
        ui->tBtnNext->setProperty("state", "disable");
    }
    else
    {
        // 中间页 mid page
        ui->tBtnFirst->setEnabled(true);
        ui->tBtnFirst->setProperty("state", "enable");
        ui->tBtnLast->setEnabled(true);
        ui->tBtnLast->setProperty("state", "enable");
        ui->tBtnPrevious->setEnabled(true);
        ui->tBtnPrevious->setProperty("state", "enable");
        ui->tBtnNext->setEnabled(true);
        ui->tBtnNext->setProperty("state", "enable");
    }
}

void PageNavigator::on_pagePerCount_currentIndexChanged(const QString &arg1)
{
    HandlePageLinesChanged(arg1.toInt());
}

void PageNavigator::on_tBtnSwitch_clicked()
{
    if (ui->lineEditPage->text().isEmpty())
    {
        return;
    }

    SetCurrentPage(ui->lineEditPage->text().toInt());
}

void PageNavigator::on_tBtnPrevious_clicked()
{
    SetCurrentPage(m_curPage - 1);
}

void PageNavigator::on_tBtnNext_clicked()
{
    SetCurrentPage(m_curPage + 1);
}

void PageNavigator::on_tBtnFirst_clicked()
{
    SetCurrentPage(1);
}

void PageNavigator::on_tBtnLast_clicked()
{
    SetCurrentPage(m_maxPage);
}

void PageNavigator::OnSearchResults(const QList<QStringList> &filteredData)
{
    int totalRecordsAfterSearch = filteredData.size();
    int pageSize = ui->pagePerCount->currentText().toInt();

    // 更新总记录数和最大页数
    m_totalRecord = totalRecordsAfterSearch;
    m_maxPage = (m_totalRecord + (pageSize - 1)) / pageSize;

    // 重置当前页为1
    m_curPage = 1;

    // 更新UI显示
    ui->labelTotalRecord->setText(QString("共有%1条记录").arg(m_totalRecord));
    ui->labelCurPage->setText(QString("第%1页/共%2页").arg(m_curPage).arg(m_maxPage));

    // 更新按钮状态
    UpdateButtonState();

    // 发出当前页改变信号
    emit SigCurrentPageChanged(m_curPage);
}
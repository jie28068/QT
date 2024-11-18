#include "controlToolPage.h"
#include "ControlToolButton.h"
#include "controlToolListView.h"
#include "controlToolListModel.h"

#include <QVBoxLayout>

ControlToolPage::ControlToolPage(QString text, QWidget *parent) : QWidget(parent), m_text(text)
{
    m_button = new ControlToolButtons(text, this);
    m_listView = new ControlToolListView(text, this);
    IconTextModel *model = new IconTextModel();
    model->addData("元件名称", "D:\\other\\image\\image\\1.jpg", false);
    model->addData("基础图层", "D:\\other\\image\\image\\2.jpg", true);
    model->addData("标注", "D:\\other\\image\\image\\3.jpg", true);
    model->addData("连接线", "D:\\other\\image\\image\\SharedScreenshot.jpg", false);
    model->addData("断电", "D:\\other\\image\\image\\白龙.jpg", false);
    m_listView->setModel(model);
    m_listView->setSpacing(4);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_button);
    layout->addWidget(m_listView);
    layout->setSpacing(0);

    setLayout(layout);
    setAttribute(Qt::WA_StyledBackground);

    // 连接model的dataChanged信号，以在状态变化时更新视图

    connect(m_listView, &QListView::clicked, [&](const QModelIndex &index)
            {

            bool isChecked = index.model()->data(index, Qt::UserRole).toBool();
              m_listView->model()->setData(index, !isChecked, Qt::UserRole);
              m_listView->update(); });
}

ControlToolPage::~ControlToolPage()
{
}

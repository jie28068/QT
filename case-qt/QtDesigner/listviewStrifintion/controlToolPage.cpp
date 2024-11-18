#include "controlToolPage.h"
#include "controlToolButton.h"
#include "controlToolListView.h"

#include <QVBoxLayout>
#include <QStandardItemModel>
ControlToolPage::ControlToolPage(QString text, QWidget *parent) : QWidget(parent), m_text(text)
{
    m_button = new ControlToolButtons(text, this);
    m_listView = new ControlToolListView(text, this);
    QStandardItemModel *model = new QStandardItemModel();
    // 添加项到模型
    for (int i = 0; i < 10; ++i)
    {
        QString text = QString("Item %1").arg(i);
        QStandardItem *item = new QStandardItem(text);
        model->appendRow(item);
    }
    m_listView->setModel(model);
    m_listView->setSpacing(4);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_button);
    layout->addWidget(m_listView);
    layout->setSpacing(0);

    setLayout(layout);
    setAcceptDrops(true);
    setAttribute(Qt::WA_StyledBackground);

    connect(m_button, &ControlToolButtons::clicked, [this]()
            { m_listView->setVisible(!m_button->getHovered()); });
}

ControlToolPage::~ControlToolPage()
{
}

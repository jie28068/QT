#include "controlToolPage.h"
#include "ControlToolButton.h"
#include "controlToolListView.h"
#include "controlToolDelegate.h"
#include "controlToolModel.h"
#include "globalDefinition.h"
#include <QDebug>
#include <QVBoxLayout>
#include <QStandardItemModel>
ControlToolPage::ControlToolPage(QString text, QWidget *parent) : QWidget(parent), m_text(text)
{
    m_button = new ControlToolButtons(text, this);
    m_listView = new ControlToolListView(text, this);
    ControlToolModel *model = new ControlToolModel(text, this);
    if (text == GlobalDefinition::controlParameters)
    {
        model->addItem(GlobalDefinition::controlPushButton, GlobalDefinition::controlParameters, QVariant());
        model->addItem(GlobalDefinition::controlComboBox, GlobalDefinition::controlParameters, QVariant());
        model->addItem(GlobalDefinition::controlLineEdit, GlobalDefinition::controlParameters, QVariant());
    }
    else if (text == GlobalDefinition::controlContainer)
    {
        model->addItem(GlobalDefinition::controlLineEdit, GlobalDefinition::controlContainer, QVariant());
    }
    else if (text == GlobalDefinition::controlDisplay)
    {
        // QPixmap("D:\\other\\image\\image\\3.jpg")
        model->addItem(GlobalDefinition::controlCheckBox, GlobalDefinition::controlDisplay, QVariant());
    }
    m_listView->setModel(model);
    m_listView->setItemDelegate(new ControlDelegate(this));
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

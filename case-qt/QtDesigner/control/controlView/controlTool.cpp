#include "controlTool.h"
#include "controlToolPage.h"

ControlTools::ControlTools(QWidget *parent) : QWidget(parent)
{
    m_contentLayout = new QVBoxLayout(this);
    m_contentLayout->setContentsMargins(0, 0, 2, 0);
    m_contentLayout->setSpacing(8);

    auto *toolBox = new QVBoxLayout(this);
    toolBox->setContentsMargins(0, 0, 2, 0);
    toolBox->addLayout(m_contentLayout);
    toolBox->addStretch(1);
    QSpacerItem *spacer = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
    toolBox->addItem(spacer);
    setLayout(toolBox);
}

ControlTools::~ControlTools()
{
}

ControlToolPage *ControlTools::addPage(QString title)
{
    ControlToolPage *page = new ControlToolPage(title, this);
    m_contentLayout->addWidget(page);
    connect(page, &ControlToolPage::clickedControlPage, this, &ControlTools::clickedControlTools);
    return nullptr;
}

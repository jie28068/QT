#include "mwidget.h"
#include <QPushButton>
#include <QTimer>

void Widget::initUI()
{
    resize(250, 200);
    verticalLayout = new QVBoxLayout(this);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    battery = new QmyBattery(this);

    verticalLayout->addWidget(battery);

    horizontalSlider = new QSlider(this);
    horizontalSlider->setValue(60);
    horizontalSlider->setOrientation(Qt::Horizontal);

    verticalLayout->addWidget(horizontalSlider);

    LabInfo = new QLabel(this);
    LabInfo->setMaximumSize(QSize(16777215, 40));

    verticalLayout->addWidget(LabInfo);

    pushButton = new QPushButton(this);
    pushButton->setText("开始充电");
    verticalLayout->addWidget(pushButton);
}

Widget::Widget(QWidget *parent) : QWidget(parent)
{
    initUI();
    m_value = 0;
    m_timer = new QTimer(this);
    m_timer->setInterval(100);
    connect(m_timer, &QTimer::timeout, this, &Widget::timer_update);
    connect(horizontalSlider, &QSlider::valueChanged, this, &Widget::on_horizontalSlider_valueChanged);
    connect(pushButton, &QPushButton::clicked, this, &Widget::on_pushButton_chiled);
}

Widget::~Widget()
{
}

void Widget::on_pushButton_chiled()
{
    battery->setPowerLevel(0);
    m_timer->start();
}

void Widget::timer_update()
{
    m_value += 2;
    battery->setPowerLevel(m_value);
    QString str = QStringLiteral("当前电量：") + QString::asprintf("%d %%", m_value);
    LabInfo->setText(str);
    horizontalSlider->setValue(m_value);
    if (m_value >= 100)
    {
        m_value = 0;
        m_timer->stop();
    }
}

void Widget::on_horizontalSlider_valueChanged(int value)
{
    battery->setPowerLevel(value);
    QString str = QStringLiteral("当前电量：") + QString::asprintf("%d %%", value);
    LabInfo->setText(str);
}
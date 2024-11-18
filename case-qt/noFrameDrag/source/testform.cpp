#include "testform.h"
#include <QVBoxLayout>
#include <QLabel>

TestForm::TestForm(QWidget *parent) : FramelessWindow(parent)
{
    initUI();

    connect(pushButton_max, &QPushButton::clicked, this, &QWidget::showMaximized);
    connect(pushButton_normal, &QPushButton::clicked, this, &QWidget::showNormal);
    connect(pushButton_min, &QPushButton::clicked, this, &QWidget::showMinimized);
}

TestForm::~TestForm()
{
}

void TestForm::initUI()
{
    resize(600, 480);
    QLabel *label = new QLabel("我是一个无边框的Dialog弹窗", this);
    pushButton_max = new QPushButton("最大化", this);
    pushButton_normal = new QPushButton("正常", this);
    pushButton_min = new QPushButton("最小化", this);
    auto pushButton_close = new QPushButton("退出", this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    QBoxLayout *layouth = new QHBoxLayout(this);
    layouth->addStretch(1);
    layouth->addWidget(label);
    layouth->addStretch(1);
    layout->addLayout(layouth);
    layout->addWidget(pushButton_normal);
    layout->addWidget(pushButton_max);
    layout->addWidget(pushButton_min);
    layout->addWidget(pushButton_close);
    setLayout(layout);

    connect(pushButton_close, &QPushButton::clicked, [&]()
            { close(); });
}

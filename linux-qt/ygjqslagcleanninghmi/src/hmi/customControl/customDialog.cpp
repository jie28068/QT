#include "customDialog.h"
CustomDialog::CustomDialog(const QString &message, globals::DialogType type, QWidget *parent) : QDialog(parent)
{

    setMinimumSize(400, 200);
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    setStyleSheet("QDialog { border-image: url(:/resource/svg/作业次数-背景框.svg); } "
                  "QLabel { color: white; font-size: 16pt; margin: 10px; background-color: transparent; } "
                  "QPushButton { font-size: 14pt; border: 2px solid white; border-radius: 10px; padding: 10px; margin: 10px; background-color: transparent; color: white; } ");

    QString str = type != globals::WarningDialog ? QString("请再次确认，是否选择%1").arg(message) : QString("%1失败！").arg(message);
    QLabel *label = new QLabel(str, this);
    label->setWordWrap(true); // 允许自动换行
    label->setAlignment(Qt::AlignCenter);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(label);
    // mainLayout->addStretch(1); // 添加一个伸缩因子，使标签位于顶部

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();

    GeneralButton *okButton = nullptr;
    GeneralButton *yesButton = nullptr;
    GeneralButton *noButton = nullptr;

    if (type == globals::DialogType::WarningDialog)
    {
        okButton = new GeneralButton("确认", this);
        buttonLayout->addWidget(okButton);
    }
    else if (type == globals::DialogType::ConfirmDialog)
    {
        yesButton = new GeneralButton("是", this);
        noButton = new GeneralButton("否", this);
        buttonLayout->addWidget(yesButton);
        buttonLayout->addWidget(noButton);
    }

    mainLayout->addLayout(buttonLayout);

    if (okButton)
        connect(okButton, &GeneralButton::clicked, this, &QDialog::accept);
    if (yesButton)
        connect(yesButton, &GeneralButton::clicked, this, &QDialog::accept);
    if (noButton)
        connect(noButton, &GeneralButton::clicked, this, &QDialog::reject);

    // 居中
    this->adjustSize();
    this->move(parent->geometry().center() - this->rect().center());
}

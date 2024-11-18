#include "generalbutton.h"

GeneralButton::GeneralButton(QWidget *parent) : QPushButton(parent)
{
    initUI();
}

GeneralButton::GeneralButton(const QString &text, QWidget *parent)
{
    initUI();
    setText(text);
}

void GeneralButton::initUI()
{
    setStyleSheet(R"(
                QPushButton{
                    color: rgb(255, 255, 255);
                    border-image: url(:/resource/page1/mipmap-xxxhdpi/按钮-默认-背景.png);
                }
                QPushButton:hover{
                    border-image: url(:/resource/page1/mipmap-xxxhdpi/按钮-激活-背景.png);
                }
                QPushButton:pressed{
                    border-image: url(:/resource/page1/mipmap-xxxhdpi/按钮-激活-背景.png);
                }
                QPushButton#pushButton_start_stop[robotState='stop']{
                    color: rgb(255, 255, 255);
                    border-image: url(:/resource/shuikoushan/急停按钮-背景.png);
                }
                QPushButton#pushButton_start_stop[robotState='stop']:hover{
                    border-image: url(:/resource/shuikoushan/急停按钮-激活.png);
                }
                QPushButton#pushButton_start_stop[robotState='stop']:pressed{
                    border-image: url(:/resource/shuikoushan/急停按钮-激活.png);
                }
            )");
}

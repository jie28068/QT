#ifndef SWITCHBUTTON_H
#define SWITCHBUTTON_H

#include <QWidget>
#include <QPushButton>
#include <QPaintEvent>

class SwitchButton : public QPushButton
{
    Q_OBJECT

public:
    explicit SwitchButton(QWidget *parent = nullptr);

signals:
    void modeChanged(bool isAutoMode); // 发射信号，通知模式改变

public slots:
    void toggleMode(bool falg); // 切换模式的槽函数

private:
    bool m_isAutoMode; // 当前是否是自动模式

    // 重写绘制事件来自定义外观
    void paintEvent(QPaintEvent *event) override;
};

#endif // SWITCHBUTTON_H

#include "SwitchButton.h"
#include <QPainter>
#include <QMouseEvent>
#include <QRadialGradient>

SwitchButton::SwitchButton(QWidget *parent) : QPushButton(parent), m_isAutoMode(true)
{
    // 设置按钮样式为无边框
    setStyleSheet("border: none;");

    // 连接点击信号到切换模式的槽函数
    connect(this, &QPushButton::clicked, this, &SwitchButton::toggleMode);
}

void SwitchButton::toggleMode()
{
    m_isAutoMode = !m_isAutoMode;
    update();
    emit modeChanged(m_isAutoMode); // 发射模式改变信号
}

void SwitchButton::paintEvent(QPaintEvent *event)
{
    QPushButton::paintEvent(event); // 调用基类的绘制事件

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 定义滚珠的半径
    qreal ballRadius = height() * 0.4; // 可以根据需要调整滚珠大小
    qreal padding = (height() - ballRadius * 2) / 2; // 按钮内部边距

    // 绘制按钮的背景
    QRectF rect(padding, padding, width() - padding * 2, height() - padding * 2);
    painter.setPen(Qt::NoPen);
    if(m_isAutoMode){
        painter.setBrush(QColor(33, 51, 107));
    }else{
        painter.setBrush(QColor(25, 127, 224));
    }
    painter.drawRoundedRect(rect, ballRadius, ballRadius); // 使用滚珠半径作为圆角半径

    // 绘制圆形滚珠
    qreal ballPosition = m_isAutoMode ? rect.width() - ballRadius * 2 : 0;
    QRectF ballRect(ballPosition + padding, padding, ballRadius * 2, ballRadius * 2);

    // 设置滚珠的渐变色
    qreal gradientCenterX = ballPosition + padding + ballRadius;
    qreal gradientCenterY = padding + ballRadius;
    QRadialGradient gradient(gradientCenterX, gradientCenterY, ballRadius);
    gradient.setColorAt(0, QColor(25, 127, 224));
    gradient.setColorAt(1, QColor(33, 51, 107));
    painter.setBrush(gradient);
    painter.drawEllipse(ballRect);

    // 绘制文本
    painter.setPen(Qt::white);
    QString text = m_isAutoMode ? "自动模式" : "手动模式";
    painter.drawText(rect, Qt::AlignCenter, text);
}

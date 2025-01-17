#include "controlToolButton.h"

#include <QPalette>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QDebug>

ControlToolButtons::ControlToolButtons(QString text, QWidget *parnt) : QWidget(parnt), m_text(text), m_isPressed(false), m_isHovered(false)
{
    setAutoFillBackground(true);
    setMinimumHeight(30);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(qRgb(235, 243, 254)));
    setPalette(palette);
}

ControlToolButtons::~ControlToolButtons()
{
}

void ControlToolButtons::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.fillRect(event->rect(), QColor(qRgb(235, 243, 254)));

    QPen pen;
    QColor colorNormal("#005A92");
    QColor colorPressed("#036EB7");
    pen.setWidth(1);
    pen.setColor(m_isPressed ? colorPressed : colorNormal);
    painter.setPen(pen);
    QFont font("Microsoft YaHei");
    font.setPixelSize(14);
    font.setBold(true);
    painter.setFont(font);
    int w = QFontMetrics(font).width(m_text);
    int h = QFontMetrics(font).height();
    QRect rect(10, (height() - h) / 2, w + 1, h + 1);
    painter.drawText(rect, m_text);

    pen.setColor(QColor(206, 206, 206));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.drawRect(event->rect());
}

QSize ControlToolButtons::sizeHint() const
{
    return QSize(200, 30);
}

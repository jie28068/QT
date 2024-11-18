#include "clickableLabel.h"

ClickableLabel::ClickableLabel(QWidget *parent) : QLabel(parent)
{
    setCursor(Qt::PointingHandCursor);
    setStatus(Normal);
}

void ClickableLabel::setStatus(Status status)
{
    switch (status)
    {
    case Normal:
        setStyleSheet("QLabel { color : white; background:transparent;}");
        break;
    case Warning:
        setStyleSheet("QLabel { color : yellow; background:transparent; }");
        break;
    case Error:
        setStyleSheet("QLabel { color : red; background:transparent; }");
        break;
    }
}

void ClickableLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        emit clicked();
    }
    QLabel::mousePressEvent(event);
}
#include "MyLabel.h"

ClickableLabel::ClickableLabel(QWidget *parent)
    : QLabel(parent) {}

void ClickableLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        emit clicked();
    }
    QLabel::mousePressEvent(event);
}

ShowLabel::ShowLabel(QWidget *parent) : QLabel(parent), m_scaleValue(1.0), m_mousePoint(0, 0), m_drawPoint(0, 0), m_rectPixmap(0, 0, 0, 0), m_isMousePress(0)
{
}

void ShowLabel::resert()
{
    m_drawPoint = QPointF(0, 0);
    m_scaleValue = 1.0;
}

void ShowLabel::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    double width = this->width() * m_scaleValue;
    double height = this->height() * m_scaleValue;
    QPixmap scalePixmap = this->pixmap()->scaled(width, height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    m_rectPixmap = QRect(m_drawPoint.x(), m_drawPoint.y(), width, height);
    painter.drawPixmap(m_rectPixmap, scalePixmap);
}

void ShowLabel::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isMousePress)
    {
        int x = event->pos().x() - m_mousePoint.x();
        int y = event->pos().y() - m_mousePoint.y();
        m_mousePoint = event->pos();
        m_drawPoint = QPointF(m_drawPoint.x() + x, m_drawPoint.y() + y);
        update();
    }
}

void ShowLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_isMousePress = true;
        m_mousePoint = event->pos();
    }
}

void ShowLabel::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        resert();
        update();
    }
    if (event->button() == Qt::LeftButton)
        m_isMousePress = false;
}

void ShowLabel::wheelEvent(QWheelEvent *event)
{
    if (event->modifiers() & Qt::ControlModifier)
    {
        changeWheelValue(event->pos(), event->delta());
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void ShowLabel::resizeEvent(QResizeEvent *event)
{
    m_drawPoint = QPointF(0, 0);
    m_scaleValue = 1.0;
    update();
}

void ShowLabel::changeWheelValue(QPoint event, int numSteps)
{
    double oldScale = m_scaleValue;
    if (numSteps > 0)
    {
        m_scaleValue *= 1.1;
    }
    else
    {
        m_scaleValue *= 0.9;
    }
    if (m_scaleValue > (SCALE_MAX_VALUE))
    {
        m_scaleValue = SCALE_MAX_VALUE;
    }
    if (m_scaleValue < (SCALE_MIN_VALUE))
    {
        m_scaleValue = SCALE_MIN_VALUE;
    }

    if (m_rectPixmap.contains(event))
    {
        double x = m_drawPoint.x() - (event.x() - m_drawPoint.x()) / m_rectPixmap.width() * (this->width() * (m_scaleValue - oldScale));
        double y = m_drawPoint.y() - (event.y() - m_drawPoint.y()) / m_rectPixmap.height() * (this->height() * (m_scaleValue - oldScale));
        m_drawPoint = QPointF(x, y);
    }
    else
    {
        double x = m_drawPoint.x() - (this->width() * (m_scaleValue - oldScale)) / 2;
        double y = m_drawPoint.y() - (this->height() * (m_scaleValue - oldScale)) / 2;
        m_drawPoint = QPointF(x, y);
    }
    update();
}
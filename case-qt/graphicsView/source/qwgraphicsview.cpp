#include "../heard/qwgraphicsview.h"
#include <QMouseEvent>
#include <QDragEnterEvent>
#include <QMimeData>
#include <QGraphicsPixmapItem>

MyGraphicsView::MyGraphicsView(QWidget *parent) : QGraphicsView(parent), m_startPos(0, 0), m_isDrawing(false)
{
}

void MyGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
    emit mouseMovePoint(event);
    QGraphicsView::mouseMoveEvent(event);
}

void MyGraphicsView::mousePressEvent(QMouseEvent *event)
{
    if (m_status == Gloabine::custom)
    {
        if (event->button() == Qt::LeftButton)
        {
            m_startPos = event->pos();
            m_isDrawing = true;
        }
    }
    if (event->button() == Qt::LeftButton)
    {
        emit mouseClicked(event);
    }
    if (event->button() == Qt::RightButton)
    {
        emit mouseClickedRight(event);
    }
    QGraphicsView::mousePressEvent(event);
}

void MyGraphicsView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        emit mouseDoubleClick(event);
    }
    QGraphicsView::mouseDoubleClickEvent(event);
}

void MyGraphicsView::keyPressEvent(QKeyEvent *event)
{
    emit keyPress(event);
    QGraphicsView::keyPressEvent(event);
}

void MyGraphicsView::wheelEvent(QWheelEvent *event)
{
    emit onwheelEvent(event);
    QGraphicsView::wheelEvent(event);
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_status == Gloabine::custom)
    {
        if (m_isDrawing && event->button() == Qt::LeftButton)
        {
            int dx = event->x() - m_startPos.x();
            int dy = event->y() - m_startPos.y();
            QRectF rect(m_startPos.x(), m_startPos.y(), dx, dy);
            scene()->addItem(new QGraphicsRectItem(rect));
        }
    }
}
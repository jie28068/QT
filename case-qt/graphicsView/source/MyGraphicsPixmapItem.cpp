#include "../heard/MyGraphicsPixmapItem.h"
#include <QGraphicsSceneMouseEvent>

MyGraphicsPixmapItem::MyGraphicsPixmapItem(const QPixmap &pixmap, QGraphicsItem *parent) : QGraphicsPixmapItem(pixmap, parent)
{
    setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsMovable);
    setAcceptHoverEvents(true);
    m_isResizing = false;
    m_resizeHandleSize = 10;
}

void MyGraphicsPixmapItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPointF pos = event->pos();
        QRectF rect = boundingRect();
        QRectF topLeftRect = QRectF(rect.topLeft(), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
        QRectF topRightRect = QRectF(QPointF(rect.right() - m_resizeHandleSize, rect.top()), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
        QRectF bottomLeftRect = QRectF(QPointF(rect.left(), rect.bottom() - m_resizeHandleSize), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
        QRectF bottomRightRect = QRectF(QPointF(rect.right() - m_resizeHandleSize, rect.bottom() - m_resizeHandleSize), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
        if (topLeftRect.contains(pos))
        {
            m_resizeHandle = TopLeft;
            m_isResizing = true;
        }
        else if (topRightRect.contains(pos))
        {
            m_resizeHandle = TopRight;
            m_isResizing = true;
        }
        else if (bottomLeftRect.contains(pos))
        {
            m_resizeHandle = BottomLeft;
            m_isResizing = true;
        }
        else if (bottomRightRect.contains(pos))
        {
            m_resizeHandle = BottomRight;
            m_isResizing = true;
        }
        else
        {
            QGraphicsPixmapItem::mousePressEvent(event);
        }
    }
    else
    {
        QGraphicsPixmapItem::mousePressEvent(event);
    }
}

void MyGraphicsPixmapItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_isResizing)
    {
        QRectF rect = boundingRect();
        QPointF pos = event->pos();
        switch (m_resizeHandle)
        {
        case TopLeft:
            setPos(pos);
            setPixmap(pixmap().scaled(rect.right() - pos.x(), rect.bottom() - pos.y(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            break;
        case TopRight:
            setPos(rect.left(), pos.y());
            setPixmap(pixmap().scaled(pos.x() - rect.left(), rect.bottom() - pos.y(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            break;
        case BottomLeft:
            setPos(pos.x(), rect.top());
            setPixmap(pixmap().scaled(rect.right() - pos.x(), pos.y() - rect.top(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            break;
        case BottomRight:
            setPixmap(pixmap().scaled(pos.x() - rect.left(), pos.y() - rect.top(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            break;
        }
    }
    else
    {
        QGraphicsPixmapItem::mouseMoveEvent(event);
    }
}

void MyGraphicsPixmapItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    m_isResizing = false;
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}

void MyGraphicsPixmapItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    QPointF pos = event->pos();
    QRectF rect = boundingRect();
    QRectF topLeftRect = QRectF(rect.topLeft(), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
    QRectF topRightRect = QRectF(QPointF(rect.right() - m_resizeHandleSize, rect.top()), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
    QRectF bottomLeftRect = QRectF(QPointF(rect.left(), rect.bottom() - m_resizeHandleSize), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
    QRectF bottomRightRect = QRectF(QPointF(rect.right() - m_resizeHandleSize, rect.bottom() - m_resizeHandleSize), QSizeF(m_resizeHandleSize, m_resizeHandleSize));
    if (topLeftRect.contains(pos) || bottomRightRect.contains(pos))
    {
        setCursor(Qt::SizeFDiagCursor);
    }
    else if (topRightRect.contains(pos) || bottomLeftRect.contains(pos))
    {
        setCursor(Qt::SizeBDiagCursor);
    }
    else
    {
        setCursor(Qt::ArrowCursor);
    }
}
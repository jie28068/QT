#if 1
#include "../heard/MyQGraphicsItem.h"
#include <QPainter>
#include <QGraphicsView>
#include <QGraphicsSceneMouseEvent>

MyQGraphicsItem::MyQGraphicsItem(QGraphicsView *view, Gloabine::PaintModel model, QGraphicsItem *parent) : QGraphicsItem(parent), m_model(model), m_view(view)
{
}

void MyQGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setPen(Qt::blue);
    painter->setBrush(Qt::NoBrush);
    painter->setOpacity(1.0);
    switch (m_model)
    {
    case Gloabine::PaintModel::rectangle:
        break;
    case Gloabine::PaintModel::triangle:
        break;
    case Gloabine::PaintModel::round:
        break;
    case Gloabine::PaintModel::line:
        break;
    default:
        break;
    }
}

QPainterPath MyQGraphicsItem::shape() const
{
    QPainterPath path;
    switch (m_model)
    {
    case Gloabine::PaintModel::rectangle:
    case Gloabine::PaintModel::line:
        path.addPolygon(cachePoints);
        break;
    case Gloabine::PaintModel::triangle:
        path.addPolygon(cachePoints);
        path.closeSubpath();
        break;
    case Gloabine::PaintModel::round:
        path.addEllipse(cachePoints.boundingRect());
        break;
    default:
        break;
    }
    return path;
}
void MyQGraphicsItem::mousePressEvent(QPoint point)
{
    QPointF scenePos = m_view->mapToScene(point);
    qreal xpos = qRound(scenePos.rx() * 1.0 / 10) * 10;
    qreal ypos = qRound(scenePos.ry() * 1.0 / 10) * 10;
    QPointF alignPos(xpos, ypos);
    switch (m_model)
    {
    case Gloabine::PaintModel::rectangle:
        if (cachePoints.isEmpty())
        {
            cachePoints.push_back(alignPos);
            cachePoints.push_back(alignPos);
        }
        break;
    case Gloabine::PaintModel::line:
        break;
    case Gloabine::PaintModel::triangle:
        break;
    case Gloabine::PaintModel::round:
        break;
    default:
        break;
    }
}

void MyQGraphicsItem::mouseMoveEvent(QPoint point)
{
    QPointF scenePos = m_view->mapToScene(point);
    qreal xpos = qRound(scenePos.rx() * 1.0 / 10) * 10;
    qreal ypos = qRound(scenePos.ry() * 1.0 / 10) * 10;
    QPointF alignPos(xpos, ypos);
    switch (m_model)
    {
    case Gloabine::PaintModel::rectangle:
        cachePoints.last() = alignPos;
        break;
    case Gloabine::PaintModel::line:
        break;
    case Gloabine::PaintModel::triangle:
        break;
    case Gloabine::PaintModel::round:
        break;
    default:
        break;
    }
}

void MyQGraphicsItem::mouseReleaseEvent(QPoint point)
{
    update();
}

void MyQGraphicsItem::mouseDoubleClickEvent(QPoint point)
{
}

#endif
#if 1
#ifndef MYQGRAPHICSITEM_H
#define MYQGRAPHICSITEM_H
#include "Globine.h"
#include <QGraphicsItem>

class MyQGraphicsItem : public QGraphicsItem
{
public:
    MyQGraphicsItem(QGraphicsView *view, Gloabine::PaintModel model, QGraphicsItem *parent = nullptr);

public:
    QRectF boundingRect() const
    {
        return shape().controlPointRect();
    }
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr);
    virtual QPainterPath shape() const;
    virtual void mousePressEvent(QPoint point);
    virtual void mouseMoveEvent(QPoint point);
    virtual void mouseReleaseEvent(QPoint point);
    virtual void mouseDoubleClickEvent(QPoint point);

private:
    QPolygonF cachePoints;
    Gloabine::PaintModel m_model;
    QGraphicsView *m_view;
};
#endif
#endif
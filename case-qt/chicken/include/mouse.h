
#ifndef MOUSE_H
#define MOUSE_H

#include <QGraphicsItem>
#include <QPainter>
#include "define.h"
class Mouse : public QGraphicsItem
{
public:
    Mouse();

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

protected:
    void advance(int step) override;

private:
    qreal angle;
    qreal speed;
    qreal mouseEyeDirection;
    QColor color;
};
#endif
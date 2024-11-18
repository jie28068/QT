#ifndef RICE_H
#define RICE_H

#include <QGraphicsItem>
#include <QPainter>

class Rice : public QGraphicsItem
{
private:
    /* data */
public:
    Rice(/* args */);
    ~Rice();

protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

private:
    QColor bordcolor;
    QColor othcolor;
};

#endif
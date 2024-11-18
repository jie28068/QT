#include "rice.h"
#include <QRandomGenerator>

Rice::Rice() : othcolor(QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256)),
               bordcolor(QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256))
{
}

Rice::~Rice()
{
}

QRectF Rice::boundingRect() const
{
    return QRectF(-30, -15, 60, 30);
}

void Rice::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    // 中间带条纹的椭圆
    QRectF ellipseRect(-10, -10, 20, 20); // 椭圆的边界矩形
    painter->setBrush(bordcolor);         // 设置画刷颜色和填充
    painter->drawEllipse(ellipseRect);    // 绘制椭圆

    // 在椭圆中心绘制条纹
    painter->setPen(Qt::white); // 设置条纹颜色
    for (int i = -3; i < 5; i += 2)
    {
        painter->drawLine(i, -10, i, 10); // 绘制垂直条纹
    }

    // 在椭圆两侧绘制结
    painter->setBrush(othcolor); // 设置结的颜色
    QPointF leftKnot(-20, 0);
    QPointF rightKnot(20, 0);
    QRectF knotRect(0, 0, 10, 10); // 结的边界矩形

    knotRect.moveCenter(leftKnot);  // 移动到椭圆左侧
    painter->drawEllipse(knotRect); // 绘制左侧的结
    knotRect.moveCenter(rightKnot); // 移动到椭圆右侧
    painter->drawEllipse(knotRect); // 绘制右侧的结
}
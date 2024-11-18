#include "mouse.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QRandomGenerator>
#include <QStyleOption>
#include <qmath.h>

const qreal Pi = M_PI;
const qreal TwoPi = 2 * M_PI;

static qreal normalizeAngle(qreal angle)
{
    while (angle < 0)
        angle += TwoPi;
    while (angle > TwoPi)
        angle -= TwoPi;
    return angle;
}

Mouse::Mouse()
    : angle(0), speed(0), mouseEyeDirection(0),
      color(QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256))
{
    setRotation(QRandomGenerator::global()->bounded(360 * 16));
}

QRectF Mouse::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

QPainterPath Mouse::shape() const
{
    QPainterPath path;
    path.addRect(-10, -20, 20, 40);
    return path;
}

void Mouse::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // 身体
    painter->setBrush(color);
    painter->drawEllipse(-10, -20, 20, 40);

    // 眼睛
    painter->setBrush(Qt::white);
    painter->drawEllipse(-10, -17, 8, 8);
    painter->drawEllipse(2, -17, 8, 8);

    // 鼻子
    painter->setBrush(Qt::black);
    painter->drawEllipse(QRectF(-2, -22, 4, 4));

    // 瞳孔
    painter->drawEllipse(QRectF(-8.0 + mouseEyeDirection, -17, 4, 4));
    painter->drawEllipse(QRectF(4.0 + mouseEyeDirection, -17, 4, 4));

    // 耳朵
    painter->setBrush(scene()->collidingItems(this).isEmpty() ? Qt::darkYellow : Qt::red);
    painter->drawEllipse(-17, -12, 16, 16);
    painter->drawEllipse(1, -12, 16, 16);

    // 尾巴
    QPainterPath path(QPointF(0, 20));
    path.cubicTo(-5, 22, -5, 22, 0, 25);
    path.cubicTo(5, 27, 5, 32, 0, 30);
    path.cubicTo(-5, 32, -5, 42, 0, 35);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path);
}

void Mouse::advance(int step)
{
    if (!step)
        return;
    // 不要移动得太远
    QLineF lineToCenter(QPointF(0, 0), mapFromScene(0, 0)); // 当前位置到场景中心的线
    if (lineToCenter.length() > 150)                        // 如果与中心的距离大于150
    {
        // 计算向中心的角度
        qreal angleToCenter = std::atan2(lineToCenter.dy(), lineToCenter.dx());
        angleToCenter = normalizeAngle((Pi - angleToCenter) + Pi / 2); // 规范化角度

        if (angleToCenter < Pi && angleToCenter > Pi / 4) // 根据角度旋转，避免走得太远
        {
            // 向左旋转
            angle += (angle < -Pi / 2) ? 0.25 : -0.25;
        }
        else if (angleToCenter >= Pi && angleToCenter < (Pi + Pi / 2 + Pi / 4))
        {
            // 向右旋转
            angle += (angle < Pi / 2) ? 0.25 : -0.25;
        }
    }
    else if (::sin(angle) < 0)
    {
        angle += 0.25; // 如果角度的sin值小于0，角度增加
    }
    else if (::sin(angle) > 0)
    {
        angle -= 0.25; // 如果角度的sin值大于0，角度减少
    }

    // 尝试避免和其他老鼠碰撞,dangerMice列表将包含所有在当前老鼠前方和下方三角形区域内的老鼠图形项
    QList<QGraphicsItem *> dangerMice = scene()->items(QPolygonF()
                                                       << mapToScene(0, 0)
                                                       << mapToScene(-30, -50)
                                                       << mapToScene(30, -50)); // 这个多边形是由三个点组成的三角形
    foreach (QGraphicsItem *item, dangerMice)
    {
        if (item == this) // 跳过自身
            continue;
        // 计算到其他老鼠的角度
        QLineF lineToMouse(QPointF(0, 0), mapFromItem(item, 0, 0));
        qreal angleToMouse = std::atan2(lineToMouse.dy(), lineToMouse.dx());
        angleToMouse = normalizeAngle((Pi - angleToMouse) + Pi / 2);
        // 根据角度避免碰撞
        if (angleToMouse >= 0 && angleToMouse < Pi / 2)
        {
            // 向右旋转
            angle += 0.5;
        }
        else if (angleToMouse <= TwoPi && angleToMouse > (TwoPi - Pi / 2))
        {
            // 向左旋转
            angle -= 0.5;
        }
    }

    // 添加一些随机运动
    if (dangerMice.size() > 1 && QRandomGenerator::global()->bounded(10) == 0)
    {
        if (QRandomGenerator::global()->bounded(1))
            angle += QRandomGenerator::global()->bounded(1 / 500.0); // 随机向一个方向转动一点
        else
            angle -= QRandomGenerator::global()->bounded(1 / 500.0); // 随机向另一个方向转动一点
    }

    speed += (-50 + QRandomGenerator::global()->bounded(100)) / 100.0; // 更新速度，增加一些随机变化

    qreal dx = ::sin(angle) * 10;                        // 根据角度计算X方向的移动距离
    mouseEyeDirection = (qAbs(dx / 5) < 1) ? 0 : dx / 5; // 计算移动变量和眼睛方向

    setRotation(rotation() + dx);                  // 设置老鼠的旋转角度
    setPos(mapToParent(0, -(3 + sin(speed) * 3))); // 设置老鼠的位置

    // 移动后检查是否触碰到B(注意迭代器删除的问题)
    QMutableListIterator<QGraphicsItem *> i(DfinePro::itemsRice);
    while (i.hasNext())
    {
        QGraphicsItem *item = i.next();
        if (collidesWithItem(item))
        {
            scene()->removeItem(item);
            i.remove();
            item = nullptr;
        }
    }
}

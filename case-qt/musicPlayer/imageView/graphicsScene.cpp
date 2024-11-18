#include "graphicsScene.h"
#include <QCoreApplication>

GraphicsScene::GraphicsScene(const QRectF &sceneRect)
    : QGraphicsScene(sceneRect) {}

void GraphicsScene::setBackgroundImage(const QPixmap &image)
{
    backgroundImage = image;
    invalidate(sceneRect(), QGraphicsScene::AllLayers);
}

void GraphicsScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    // 绘制背景图片
    painter->drawPixmap(rect, backgroundImage, backgroundImage.rect());
}
#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QPainter>
#include <QPixmap>

class GraphicsScene : public QGraphicsScene
{
public:
    GraphicsScene(const QRectF &sceneRect);

    void setBackgroundImage(const QPixmap &image);

protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;

private:
    QPixmap backgroundImage;
};
#endif
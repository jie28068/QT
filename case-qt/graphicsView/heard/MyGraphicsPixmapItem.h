#ifndef MYPIXMAP_H
#define MYPIXMAP_H
#include <QCursor>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneHoverEvent>

class MyGraphicsPixmapItem : public QGraphicsPixmapItem
{

public:
    MyGraphicsPixmapItem(const QPixmap &pixmap, QGraphicsItem *parent = nullptr);
    ~MyGraphicsPixmapItem() {}

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;

private:
    enum ResizeHandle
    {
        None,
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight
    };

    bool m_isResizing;
    ResizeHandle m_resizeHandle;
    int m_resizeHandleSize;
};
#endif
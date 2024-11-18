#ifndef QWGRAPHICSVIEW_H
#define QWGRAPHICSVIEW_H
#include "Globine.h"
#include <QGraphicsView>

class MyGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    MyGraphicsView(QWidget *parent = nullptr);
    void setStatus(int status) { m_status = status; }

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

signals:
    void mouseMovePoint(QMouseEvent *event);    // 鼠标移动
    void mouseClicked(QMouseEvent *event);      // 鼠标单击
    void mouseDoubleClick(QMouseEvent *event);  // 双击事件
    void keyPress(QKeyEvent *event);            // 按键事件
    void onwheelEvent(QWheelEvent *event);      // 滚轮事件
    void mouseClickedRight(QMouseEvent *event); // 右键菜单

private:
    int m_status;
    QPointF m_startPos;
    bool m_isDrawing;
};

#endif
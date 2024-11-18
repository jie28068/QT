#ifndef MYLABLE_H
#define MYLABLE_H

#include <QPixmap>
#include <QObject>
#include <QLabel>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>

class ClickableLabel : public QLabel
{
    Q_OBJECT
public:
    ClickableLabel(QWidget *parent = nullptr);

signals:
    void clicked();

protected:
    void mousePressEvent(QMouseEvent *event) override;
};

class ShowLabel : public QLabel
{
public:
    ShowLabel(QWidget *parent = nullptr);
    void resert();

protected:
    void paintEvent(QPaintEvent *) override;
    void mouseMoveEvent(QMouseEvent *ev) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseReleaseEvent(QMouseEvent *ev) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void changeWheelValue(QPoint event, int value);

private:
    double m_scaleValue;  // 图片缩放倍数
    QPointF m_drawPoint;  // 绘图起点
    QPointF m_mousePoint; // 鼠标当前位置点
    QRect m_rectPixmap;   // 被绘图片的矩形范围
    bool m_isMousePress;  // 鼠标是否按下

    const double SCALE_MAX_VALUE = 10.0; // 最大放大到原来的10倍
    const double SCALE_MIN_VALUE = 0.5;  // 最小缩小到原来的0.5倍
};

#endif
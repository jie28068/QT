#ifndef MLABEL
#define MLABEL

#include <QComboBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QPainter>
#include <QToolTip>
#include <QLabel>

#define LABLE_WIDTH 150
#define LABLE_HEIGHT 50

class MyLable : public QLabel
{
    Q_OBJECT
public:
    MyLable(QWidget *parnt = nullptr);

protected:
    void paintEvent(QPaintEvent *) override;
    void mouseMoveEvent(QMouseEvent *ev) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseReleaseEvent(QMouseEvent *ev) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void changeWheelValue(QPoint event, int value);
    bool event(QEvent *e) override;
    void contextMenuEvent(QContextMenuEvent *ev) override;

signals:
    void changePixmp();
    void delPixmp();

private:
    double m_scaleValue;  // 图片缩放倍数
    QPointF m_drawPoint;  // 绘图起点
    QPointF m_mousePoint; // 鼠标当前位置点
    QRect m_rectPixmap;   // 被绘图片的矩形范围
    bool m_isMousePress;  // 鼠标是否按下

    const double SCALE_MAX_VALUE; // 最大放大到原来的10倍
    const double SCALE_MIN_VALUE; // 最小缩小到原来的0.5倍
};

#endif
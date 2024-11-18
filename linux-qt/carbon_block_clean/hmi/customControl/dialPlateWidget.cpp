#include "dialPlateWidget.h"

const int DialPlateWidget::radius = 100;

DialPlateWidget::DialPlateWidget(QWidget *parent) : QWidget(parent)
{
    maxv = 200;
    minv = 0;
    this->installEventFilter(this);
    m_refSize = 2.5 * radius;
}

DialPlateWidget::~DialPlateWidget()
{
}

void DialPlateWidget::setValue(double ivalue)
{
    // 确保值在最小值和最大值之间
    if (ivalue < minv)
    {
        ivalue = minv;
    }
    else if (ivalue > maxv)
    {
        ivalue = maxv;
    }
    m_value = ivalue;
    percent = (ivalue - minv) / (maxv - minv) * 100;
    update();
}

double DialPlateWidget::getValue() const
{
    return minv + percent * (maxv - minv) / 100; // 根据百分比计算实际值
}

QSize DialPlateWidget::sizeHint() const
{
    return QSize(m_refSize, m_refSize);
}

QSize DialPlateWidget::minimumSizeHint() const
{
    return QSize(30, 10);
}

void DialPlateWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

    // 计算缩放比例，确保仪表盘适应Widget大小
    float scale = qMin(width(), height()) / (float)m_refSize;
    painter.scale(scale, scale);

    // 设置坐标原点为中心点
    painter.translate(width() / (2 * scale), height() / (2 * scale));

    // 绘制背景、刻度线、刻度数字、指针和文字
    drawBg(&painter);
    drawDial(&painter);
    drawScaleNum(&painter);
    drawIndicator(&painter);
    drawText(&painter);

    QWidget::paintEvent(event);
}

void DialPlateWidget::drawBg(QPainter *painter)
{
    int r = radius;
    painter->save();
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(172, 172, 172));
    painter->drawEllipse(-r, -r, r * 2, r * 2);

    r = radius * 0.9;
    painter->setBrush(QColor(40, 40, 40));
    painter->setPen(Qt::NoPen);
    painter->drawEllipse(-r, -r, r * 2, r * 2);
    painter->restore();
}

void DialPlateWidget::drawDial(QPainter *painter)
{
    int r = radius * 0.85;
    double lineWidth = 1;
    painter->save();

    painter->rotate(Angle);

    double rotate = (double)(360 - (Angle * 2)) / 100;

    for (int i = 0; i <= 100; i++)
    {
        QColor color = QColor(84, 84, 84);
        if (i <= 30)
            color = QColor(250, 0, 0);
        if (i > 30)
            color = QColor(0, 255, 0, 255);
        if ((i % 10) == 0)
        {
            painter->setPen(QPen(color, 4.2 * lineWidth));
            painter->drawLine(0, r, 0, r / 1.2);
        }
        else if ((i % 2) == 0)
        {
            painter->setPen(QPen(color, 1 * lineWidth));
            painter->drawLine(0, r, 0, r / 1.1);
        }
        painter->rotate(rotate);
    }

    painter->restore();
}

void DialPlateWidget::drawScaleNum(QPainter *painter)
{
    painter->save();

    int r = (int)(radius * 0.6); // 增加半径，使刻度数字更远离中心
    painter->setFont(QFont("Arial", 12));
    painter->setPen(QPen(QColor(255, 255, 255)));
    QFontMetricsF fm = QFontMetricsF(painter->font());

    // 计算五个标值的间隔
    double valueRange = maxv - minv;
    double valueStep = valueRange / 4.0;          // 四个间隔，五个标值
    double angleStep = (360.0 - Angle * 2) / 4.0; // 对应的角度间隔

    for (int i = 0; i <= 4; ++i)
    {
        double value = minv + i * valueStep;       // 当前标值
        double angle = 90 + Angle + i * angleStep; // 当前标值对应的角度

        float angleArc = (int(angle) % 360) * 3.14 / 180; // 转换为弧度
        int x = r * cos(angleArc);
        int y = r * sin(angleArc);

        QString speed = QString::number(value);
        int w = (int)fm.width(speed);
        int h = (int)fm.height();
        x = x - w / 2;
        y = y + h / 4; // 调整文本位置

        painter->drawText(QPointF(x, y), speed);
    }

    painter->restore();
}

void DialPlateWidget::drawIndicator(QPainter *painter)
{
    painter->save();
    QPolygon pts;
    int r = radius * 0.6;
    pts.setPoints(3, -2, 0, 2, 0, 0, r);

    double angle = Angle + (360.0 - 2 * Angle) * (percent / 100.0);

    painter->rotate(angle);

    // 绘制指针
    QRadialGradient haloGradient(0, 0, 60, 0, 0); // 辐射渐变，内部填充颜色
    haloGradient.setColorAt(0, QColor(100, 100, 100));
    haloGradient.setColorAt(1, QColor(250, 50, 50)); // red
    painter->setPen(QColor(250, 150, 150));          // 边框颜色
    painter->setBrush(haloGradient);
    painter->drawConvexPolygon(pts);

    // 绘制中心圆圈
    QRadialGradient radial(0, 0, 14); // 渐变
    radial.setColorAt(0.0, QColor(100, 100, 100));
    radial.setColorAt(1.0, QColor(250, 50, 50));
    painter->setPen(Qt::NoPen); // 填满没有边界
    painter->setBrush(radial);
    painter->drawEllipse(-7, -7, 14, 14);

    painter->restore();
}

void DialPlateWidget::drawText(QPainter *painter)
{
    painter->save();

    painter->setFont(QFont("Arial", 8));
    painter->setPen(QPen(QColor(255, 255, 255)));
    QFontMetricsF fm = QFontMetricsF(painter->font());
    QString speed = title + QString::number(m_value) + unit;
    int w = (int)fm.width(speed);
    painter->drawText(QPointF(-w / 2, (int)(0.7 * radius)), speed);
    painter->restore();
}

void DialPlateWidget::setContent(QString title, QString unit)
{
    this->unit = unit;
    this->title = title;
    update(); // 更新显示
}

double DialPlateWidget::getMinValue() const
{
    return minv;
}

void DialPlateWidget::setMinValue(double minValue)
{
    minv = minValue;
}

double DialPlateWidget::getMaxValue() const
{
    return maxv;
}

void DialPlateWidget::setMaxValue(double maxValue)
{
    maxv = maxValue;
}

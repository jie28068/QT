#pragma once

#include <QWidget>
#include <QPainter>
#include <QResizeEvent>
#include <QtMath>
#include <QCoreApplication>

class DialPlateWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(double minValue READ getMinValue WRITE setMinValue USER true)
    Q_PROPERTY(double maxValue READ getMaxValue WRITE setMaxValue USER true)
    Q_PROPERTY(double value READ getValue WRITE setValue USER true)

public:
    explicit DialPlateWidget(QWidget *parent = nullptr);
    // 获取和设置最小值
    double getMinValue() const;
    void setMinValue(double minValue);

    // 获取和设置最大值
    double getMaxValue() const;
    void setMaxValue(double maxValue);

    /// @brief  设置当前值
    void setValue(double ivalue);
    double getValue() const;

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
    ~DialPlateWidget();

private:
    void paintEvent(QPaintEvent *event) override;

    void drawBg(QPainter *painter);
    void drawDial(QPainter *painter);
    void drawScaleNum(QPainter *painter);
    void drawIndicator(QPainter *painter);
    void drawText(QPainter *painter);

public:
    void setContent(QString title, QString unit = QString());

private:
    QString unit;            // 单位
    QString title;           // 名称
    const static int radius; // 半径
    double maxv;             // 最小值
    double minv;             // 最大值
    double percent = 0;      // 百分比
    int m_refSize = 200;     // 刻度线长度
    int Angle = 45;          // 刻度线角度
    double m_value = 0;      // 当前值
};

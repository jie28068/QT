[TOC]
### 控件介绍
- ==QProgressBar==是Qt框架中提供的一个控件，用于在界面上显示任务的进度。它通常用于向用户展示一个操作完成的百分比，比如文件复制、数据加载等操作的进度。
`QProgressBar`的主要特性：
1. 范围和值：QProgressBar有一个最小值（minimum）和一个最大值（maximum），以及一个当前值（value）。进度条的显示范围从最小值到最大值，当前值表示任务的完成程度。
2. 文本显示：进度条可以显示文本，比如当前值、百分比或者自定义的文本。你可以通过`setTextVisible()`函数来控制文本的显示，以及通过`setFormat()`函数来设置文本的格式。
3. 反转方向：进度条的方向可以从左到右（默认），也可以设置为从右到左。这可以通过`setInvertedAppearance()`函数来实现。
4. 进度步长：你可以通过`setSingleStep()`函数来设置单步进度，以及通过`setPageStep()`函数来设置页步长。
5. 样式和自定义绘制：QProgressBar支持样式表，允许你通过CSS来定制进度条的外观。此外，你还可以通过继承QProgressBar并重写`paintEvent()`函数来自定义绘制进度条。
6. 信号和槽：QProgressBar提供了`valueChanged`信号，当进度条的值发生变化时发出。你可以连接这个信号到槽函数，以便在进度更新时执行特定的操作。


- ==QSlider==是Qt框架中提供的一个控件，用于允许用户通过拖动滑块来选择一个范围内的值。它通常用于音量控制、亮度调整或其他需要用户输入一个特定范围内的值的场景。QSlider可以水平或垂直显示，并且可以设置为有刻度或不显示刻度。它提供了多种信号和槽，使得它可以很容易地与其他Qt控件集成。
`QSlider`的一些主要特性：
1. 方向：QSlider可以设置为水平（`Qt::Horizontal`）或垂直（`Qt::Vertical`）。
2. 范围：可以通过`setMinimum()`和`setMaximum()`函数设置最小值和最大值。
3. 步长：可以通过`setSingleStep()`和`setPageStep()`函数设置单步和页步长。
4. 刻度位置：可以通过`setTickPosition()`函数设置刻度的位置，例如QSlider::NoTicks、QSlider::TicksAbove、QSlider::TicksBelow、QSlider::TicksBothSides和QSlider::TicksLeft。
5. 信号：QSlider提供了`valueChanged`信号，当滑块的值发生变化时发出。

- **要实现进度条关键就在会绘制函数**
- **`QProgressBar`也是继承于`QWidget`的，意味着想要自绘进度条，其实可以继承于`QWidget`，而非`QProgressBar`.**

### 绘制函数
- 图一 **继承于QWidget**
```cpp
void QmyBattery::paintEvent(QPaintEvent *event)
{ // 界面组件的绘制
    Q_UNUSED(event);

    QPainter painter(this);
    QRect rect(0, 0, width(), height()); // viewport矩形区
    painter.setViewport(rect);           // 设置Viewport
    painter.setWindow(0, 0, 120, 50);    // 设置窗口大小，逻辑坐标
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);

    // 绘制电池边框
    QPen pen;                        // 设置画笔
    pen.setWidth(2);                 // 线宽
    pen.setColor(mColorBorder);      // 划线颜色
    pen.setStyle(Qt::SolidLine);     // 线的类型，实线、虚线等
    pen.setCapStyle(Qt::FlatCap);    // 线端点样式
    pen.setJoinStyle(Qt::BevelJoin); // 线的连接点样式
    painter.setPen(pen);

    QBrush brush;                     // 设置画刷
    brush.setColor(mColorBack);       // 画刷颜色
    brush.setStyle(Qt::SolidPattern); // 画刷填充样式
    painter.setBrush(brush);

    rect.setRect(1, 1, 109, 48);
    painter.drawRect(rect); // 绘制电池边框

    brush.setColor(mColorBorder); // 画刷颜色
    painter.setBrush(brush);
    rect.setRect(110, 15, 10, 20);
    painter.drawRect(rect); // 画电池正极头

    // 画电池柱
    if (mPowerLevel > mLowLeverl)
    {                                // 正常颜色电量柱
        brush.setColor(mColorPower); // 画刷颜色
        pen.setColor(mColorPower);   // 划线颜色
    }
    else if (mPowerLevel < mWarnLevel)
    {                                  // 电量低电量柱
        brush.setColor(mColorWarning); // 画刷颜色
        pen.setColor(mColorWarning);   // 划线颜色
    }
    else
    {
        brush.setColor(mClolorLowLeverl); // 画刷颜色
        pen.setColor(mClolorLowLeverl);   // 划线颜色
    }
    painter.setBrush(brush);
    painter.setPen(pen);

    if (mPowerLevel > 0)
    {
        rect.setRect(5, 5, mPowerLevel, 40);
        painter.drawRect(rect); // 画电池柱
    }

    // 绘制电量百分比文字
    QFontMetrics textSize(this->font());
    QString powStr = QString::asprintf("%d%%", mPowerLevel);
    QRect textRect = textSize.boundingRect(powStr); // 得到字符串的rect

    painter.setFont(this->font());
    pen.setColor(mColorBorder); // 划线颜色
    painter.setPen(pen);

    painter.drawText(55 - textRect.width() / 2,
                     23 + textRect.height() / 2,
                     powStr);
}
```
- 图二 **继承于QProgressBar**
```cpp
    void paintEvent(QPaintEvent *event) override
    {
        QProgressBar::paintEvent(event); // 调用基类的paintEvent()以绘制基本的进度条
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        // 绘制圆形指示器
        int width = this->width();
        int height = this->height();
        int progress = this->value();
        int max = this->maximum();
        double ratio = double(progress) / double(max);
        int circleWidth = width * ratio;
        int circleHeight = height / 2;
        painter.setBrush(QColor("#3498db"));
        painter.drawEllipse(circleWidth - (height / 2), circleHeight - (height / 2), height, height);
    }
```
### 总结
- 知识理应共享
- [源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/pmgressbar)在此。

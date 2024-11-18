- QT 实现无边框可伸缩变换有阴影的QDialog弹窗

- **实现无标题栏窗口的拖拽移动、调节窗口大小以及边框阴影效果。**
- 初始化时进行位或操作，将这些标志合并为一个值，并将其设置为窗口的标志。这些标志分别表示这是一个对话框、无边框窗口、有标题栏、有最小化按钮和最大化按钮。

```cpp
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
    setAttribute(Qt::WA_TranslucentBackground);
```

- 重写绘制`paintEvent`函数

```cpp
void FramelessWindow::paintEvent(QPaintEvent *event)
{
    QDialog::paintEvent(event);
    if (!_shadeEnabled)
    {
        return;
    }

    // 先将窗口背景刷新为白色
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (this->isMaximized() || this->isFullScreen()) // 最大化或全屏下刷新整个窗口为白色，且让子窗口填充满
    {
        QLayout *layout = this->layout();
        if (layout != NULL)
            layout->setContentsMargins(0, 0, 0, 0);

        path.addRect(this->rect());
        painter.fillPath(path, QBrush(Qt::white));
        return;
    }
    else // 正常大小显示，让主窗口布局四周空出阴影所需宽度。
    {
        QLayout *layout = this->layout();
        if (layout != NULL)
            layout->setContentsMargins(BoardShadeWidth, BoardShadeWidth, BoardShadeWidth, BoardShadeWidth);

        path.addRect(BoardShadeWidth, BoardShadeWidth,
                     this->width() - BoardShadeWidth * 2,
                     this->height() - BoardShadeWidth * 2);
        painter.fillPath(path, QBrush(Qt::white));
    }
        painter.drawPath(path);
}

```

- 此类实现有2种方案

1. 定义**Platform_Win**，此类仅支持windows系统，**可以移动和改变大小；无法跨平台，效率高**；
2. 非定义**Platform_Win**，支持跨平台，但是**仅能实现移动，无法改变大小，效率低**；

- 定义**Platform_Win**方式需要重写`nativeEvent`函数

```cpp
    /**
     * 处理原生事件，用于实现无边框窗口的自定义拖动和缩放逻辑。
     *
     * @param eventType 事件类型。
     * @param message 事件的详细信息。
     * @param result 用于返回事件处理结果。
     * @return 如果事件被处理，则返回true；否则返回false。
     */
bool FramelessWindow::nativeEvent(const QByteArray &eventType, void *message, long *result)
{
#ifdef Platform_Win
    if (!_resizeEnabled)
    { // 如果禁止了缩放，则将事件传递给父类处理
        return QDialog::nativeEvent(eventType, message, result);
    }

    Q_UNUSED(eventType);
    MSG *msg = reinterpret_cast<MSG *>(message);
    switch (msg->message)
    {
    case WM_NCHITTEST:
        // 计算鼠标位置相对于窗口的坐标
        int xPos = GET_X_LPARAM(msg->lParam) - this->frameGeometry().x();
        int yPos = GET_Y_LPARAM(msg->lParam) - this->frameGeometry().y();
        // 如果鼠标位置上有控件，则不处理
        if (this->childAt(xPos, yPos) != NULL)
        {
            return false;
        }
        // 判断鼠标位置是否在伪标题栏区域，以及在哪个边缘或角落
        // 鼠标位置上没有控件，先判断是否位于伪标题栏
        QRect rect = this->rect();
        rect.setHeight(_doubleClickHeight);
        if (rect.contains(QPoint(xPos, yPos))) // 鼠标位置位于伪标题栏区域内
        {
            *result = HTCAPTION;
        }
        // 判断鼠标是否在窗口的四个边或四个角落上
        if (xPos > BoardStartPix && xPos < BoardEndPix)
            *result = HTLEFT;
        if (xPos > (this->width() - BoardEndPix) && xPos < (this->width() - BoardStartPix))
            *result = HTRIGHT;
        if (yPos > BoardStartPix && yPos < BoardEndPix)
            *result = HTTOP;
        if (yPos > (this->height() - BoardEndPix) && yPos < (this->height() - BoardStartPix))
            *result = HTBOTTOM;
        if (xPos > BoardStartPix && xPos < BoardEndPix && yPos > BoardStartPix && yPos < BoardEndPix)
            *result = HTTOPLEFT;
        if (xPos > (this->width() - BoardEndPix) && xPos < (this->width() - BoardStartPix) && yPos > BoardStartPix && yPos < BoardEndPix)
            *result = HTTOPRIGHT;
        if (xPos > BoardStartPix && xPos < BoardEndPix && yPos > (this->height() - BoardEndPix) && yPos < (this->height() - BoardStartPix))
            *result = HTBOTTOMLEFT;
        if (xPos > (this->width() - BoardEndPix) && xPos < (this->width() - BoardStartPix) && yPos > (this->height() - BoardEndPix) && yPos < (this->height() - BoardStartPix))
            *result = HTBOTTOMRIGHT;
        return true;
    }
    return false;
#else
    // 在非Windows平台上，将事件传递给父类处理
    return QDialog::nativeEvent(eventType, message, result);
#endif
}
```

- ==阴影效果可以使用软件的方式绘制，也可以使用图片的方式，在`paintEvent`函数实现==，相对复杂，不在此描述，可看源码。
- 知识理应共享，[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/noFrameDrag)在此。
- 这个demo也是学习的，使用了很多比较专的宏与函数，加的注释尽量写了用途，不清楚的自己去查吧。

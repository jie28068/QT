QT 使用QLsitView 实现数据的分组多级显示，子列表可动态显示/隐藏
[TOC]

### 效果图

***

### 概述

- 在QT中多级显示应该是`QTreeView`该干的事，但是当我们还需要图标显示的时候，也就是 ==setViewMode(QListView::IconMode)== ，`QTreeView`就无法满足了。
- 描述下核心思想：整体分为俩层结构，都是继承于`QWidget`。
    1. 最上层为一个`QWidget`,用于存放所有第二级的`QWidget`。
    2. 第二层为`QWidget`，就也是所看到的每一个组。里面都有一个`QListView`，用于控制隐藏/显示，看似为一个按钮，其实也是一个`QWidget`。
    3. 也就是说整体就是有多个组拼起来的，而每个组中都有一个`QListView`和~~按钮~~，从而形成多级结构。
- 图中所示虽然只有俩级，但还可以再加三级，四级....，完全可以实现树的多级显示。

***

### 部分代码

- 代码好像都没啥好描述的，主要就是实现思路，**自己去拉源码看吧**。这里就贴下~~按钮~~的绘制吧。

```cpp
void ControlToolButtons::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.fillRect(event->rect(), QColor(qRgb(235, 243, 254)));

    QPen pen;
    QColor colorNormal("#005A92");
    QColor colorPressed("#036EB7");
    pen.setWidth(1);
    pen.setColor(m_isPressed ? colorPressed : colorNormal);
    painter.setPen(pen);
    QFont font("Microsoft YaHei");
    font.setPixelSize(14);
    font.setBold(true);
    painter.setFont(font);
    int w = QFontMetrics(font).width(m_text);
    int h = QFontMetrics(font).height();
    QRect rect(10, (height() - h) / 2, w + 1, h + 1);
    painter.drawText(rect, m_text);

    QPainterPath path;
    int width = event->rect().width();
    if (!m_isHovered)
    { // 展开
        QPoint pos1(width - 10, 12);
        QPoint pos2(width - 20, 12);
        QPoint pos3(width - 15, 17);
        path.moveTo(pos1);
        path.lineTo(pos2);
        path.lineTo(pos3);
        path.lineTo(pos1);
        painter.fillPath(path, m_isPressed ? colorPressed : colorNormal);
    }
    else
    { // 收缩
        QPoint pos1(width - 20, 12);
        QPoint pos2(width - 20, 18);
        QPoint pos3(width - 12, 15);
        path.moveTo(pos1);
        path.lineTo(pos2);
        path.lineTo(pos3);
        path.lineTo(pos1);
        painter.fillPath(path, m_isPressed ? colorPressed : colorNormal);
    }
    pen.setColor(QColor(206, 206, 206));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.drawRect(event->rect());
}
```

***

### 总结

- 知识理应共享,[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/QtDesigner/listviewStrifintion)在此。
- 这个示例中的功能点，还是比较简单的，只要你理解了思路，应该可以轻松实现。

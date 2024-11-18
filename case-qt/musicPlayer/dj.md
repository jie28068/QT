[TOC]
QT 实现音乐播放器【二】 歌词同步+滚动+特效

### 效果图

***

### 概述

- 先整体说明一下这个效果的实现，你所看到的歌词都是QGraphicsObject，在QGraphicsView上绘制(paint)出来的。也就是说每一句歌词都是一个图元(item)。

1. 为什么用QGraphicsView框架？
    - 在做歌词滚动效果时，我看网上实现这一效果基本上都是用QLabel，这样或许简单很多，但是效果单一，且不够灵活。使用图形视图这套，图形项可以自由地被移动、缩放、旋转和编辑。当然主要还是为了提升自己，可以更熟悉这套框架。

2. 如何解析歌词？
    - 这里解析的是lrc文件为一般的歌词文件，格式如下：格式是固定的，那么就可以通过正则表达式来解析。然后存放在一个`QMap`中，key是时间，value是歌词。

    ```cpp
    [02:08.496]乌蒙山连着山外山
    [02:11.138]月光洒下了响水滩
    [02:13.993]有没有人能告诉我
    [02:16.487]可是苍天对你在呼唤
    ```

3. 如何同步歌词？
    - `QMediaPlayer`中有一个信号`positionChanged`,播放音乐时会时刻刻触发，可以获取当前播放时间。然后和前面我们存放在`QMap`中的时间进行对比，所以`QMap`存放的时间格式要按`positionChanged`发出的时间格式来解析。但我试验过很多次俩者的时间都是无法精确相等的。**这里采取的方案是遍历QMap，找到第一个时间大于等于`positionChanged`发出的时间，然后获取这个时间对应的歌词，这便是当前的歌词**。然后通过当前的key在获取前后几句的歌词。

4. 歌词滚动以及那些特效如何实现的？
    - 同步歌词的时候会获取当前歌词以及前后几句歌词，提前存好对应歌词的特效，如下：这里面存了一个`QMap`，里面存放了每一句歌词的属性，包括字体大小，位置，透明度等等。

    ```cpp
     m_textMapInfolst << QMap<QString, QString>{
    {"index", "1"},
    {"font", "12"},
    {"y", "-100"},
    {"x", "300"},
    {"opacity", "0.2"}};
    ```

    我这里有七句歌词，那么就存七个`QMap`在一个`QList`中，当歌词刷新的时候就去遍历，根据`QMap`中的属性来设置`item`歌词,这里的图元要自己实现，重写`paint`函数。

***

### 代码

#### 解析歌词

- 解析的时候把格式设置`GB 2312`，不然会是乱码。按行已经`QMediaPlayer`的时间格式读取数据，并全部存放到`listLyricsMap`中。

```cpp
bool Lyrics::readLyricsFile(QString lyricsPath)
{
    listLyricsMap.clear();
    QFile file(lyricsPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        listLyricsMap.clear();
        return false;
    }
    QTextStream in(&file);
    in.setCodec("GB 2312");
    QString line;
    while (!in.atEnd())
    {
        line = in.readLine();
        analysisLyricsFile(line);
    }

    return true;
}

bool Lyrics::analysisLyricsFile(QString line)
{
    if (line == NULL || line.isEmpty())
    {
        return false;
    }
    QRegExp timeRegExp("\\[(\\d+):(\\d+\\.\\d+)\\]");

    if (timeRegExp.indexIn(line) != -1)
    {
        qint64 totalTime = timeRegExp.cap(1).toInt() * 60000 + // 分钟
                           timeRegExp.cap(2).toFloat() * 1000; // 秒

        QString lyricText = line.mid(timeRegExp.matchedLength());
        listLyricsMap.insert(totalTime, lyricText);
    }
    return true;
}
```

#### 歌词同步

- 绑定信号

```cpp
    connect(player, SIGNAL(positionChanged(qint64)),
            this, SLOT(updateTextTime(qint64)));
```

- 读取对应`listLyricsMap`中的歌词

```cpp

void MainWindow::updateTextTime(qint64 position)
{
    auto lrcMap = lyric->getListLyricsMap();
    qint64 previousTime = 0;
    qint64 currentLyricTime = 0;
    QMapIterator<qint64, QString> i(lrcMap);
    while (i.hasNext())
    {
        i.next();
        if (position < i.key())
        {
            QString currentLyric = lrcMap.value(previousTime);
            currentLyricTime = previousTime;
            break;
        }
        previousTime = i.key();
    }

    QStringList displayLyrics; // 存储将要显示的歌词列表。
    // 获取将要显示的歌词
    QMap<qint64, QString>::iterator it = lrcMap.find(currentLyricTime);
    // 显示前三句，如果it不是开头，就向前移动迭代器
    for (int i = 0; i < 3 && it != lrcMap.begin(); i++)
    {
        --it;
        displayLyrics.prepend(it.value());
    }

    // 重置迭代器
    it = lrcMap.find(currentLyricTime);
    QString currntStr = QString();
    // 显示当前句
    if (it != lrcMap.end())
    {
        currntStr = QString("<font color='red'>" + it.value() + "</font>");
        displayLyrics.append(it.value());
    }

    // 显示后三句
    for (int i = 0; i < 3 && it != lrcMap.end(); i++)
    {
        ++it;
        if (it != lrcMap.end())
        {
            displayLyrics.append(it.value());
        }
    }
    //更新显示
    imageViewWindow->textChanged(displayLyrics);
}
```

#### 歌词特效

- 同步于歌词的改动，清空场景遍历特效`m_textMapInfolst`,重新进行图元绘制

```cpp
void ImageViewWindow::textChanged(QStringList &lsit)
{
 m_scene->clear();
 for (int index = 0; index < m_textMapInfolst.size(); index++)
 {
  const auto textInfoMap = m_textMapInfolst[index];
  GraphicsText *item = new GraphicsText();
  item->setStr(lsit[index]);
  item->setStrFont(textInfoMap["font"].toInt());
  item->setItemOffset(QPointF(textInfoMap["x"].toInt() + image_xoffset, textInfoMap["y"].toInt() + image_yoffset));
  item->setZValue(textInfoMap["index"].toInt());
  item->setOpacity(textInfoMap["opacity"].toFloat());
  m_items << item;
  m_scene->addItem(item);
 }
}
```

- 自绘图元

```cpp
void GraphicsText::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
 painter->setFont(m_font);
 if (m_font.pointSize() > 20)
 {
  painter->setPen(QPen(Qt::red));
 }
 else
 {
  painter->setPen(QPen(Qt::blue));
 }
 painter->drawText(offset, str);
}
```

***

### 总结

- 实现这个功能遇到的问题挺多的，比如绘制文本的时候只有一根线显示，是要`view`设置`setViewportUpdateMode(QGraphicsView::FullViewportUpdate)`，类似的问题挺多，还不好找。
- 歌词特效这块还可以再扩展，字体，入场效果等都可以设置。
- 当然这个功能还有很多可以优化的地方，BUG或许也不少，实现标题的功能的逻辑就是如上，可以作为参考。

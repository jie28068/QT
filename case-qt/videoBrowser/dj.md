[TOC]
QT 使用QMediaPlayer实现的简易视频播放器
### 效果图
### 功能点
1. **播放指定视频**
2. **全屏/退出全屏**
3. **开始/暂停/重置视频**
4. **拖拽到指定位置播放**
### 类介绍
- 需要在配置文件中加入`Multimedia`, `MultimediaWidgets`这俩个库。
- `Multimedia`：提供了一套用于处理音频、视频、摄像头和广播数据的API。
- `MultimediaWidgets`：提供了一些与多媒体相关的图形界面组件。
- `QVideoProbe`是Qt多媒体模块中的一个类，它用于监控视频流的输出。这个类允许你接收视频帧的数据，而不需要直接与视频输出设备交互。
- `QMediaPlayer` 使用生产者-消费者模型来处理媒体内容。它从媒体源（如文件或网络流）获取数据，然后通过播放控制接口（如播放、暂停、停止）和播放状态接口（如当前播放位置、总时长）来控制媒体内容的播放。
- 使用 `QMediaPlayer`时，通常需要将其与一个或多个媒体输出组件结合使用，例如`QVideoWidget`用于视频播放，`QAudioOutput`用于音频播放。
==注意==：在==Qt6==中使用`QMediaPlayer`时，使用的是`setSource`函数设置视频资源，而==Qt5==中并没有这个函数，使用的是`setMedia`函数。而且有个非常坑的地方，==Qt6==设置完`QVideoWidget`直接使用没有问题，而==Qt5==就会存在问题。
信息栏会报错：
```
DirectShowPlayerService::doRender: Unresolved error code 0x80040266 
```
**这极具迷惑性，当去百度时，你就会看到一堆让你安装解码器的，安装后并没有什么用**。
最后看到一个官方的示例才解决，需要设置`QVideoProbe`。
```cpp
    m_player = new QMediaPlayer(this);
    m_videoProbe = new QVideoProbe(this);
    m_videoProbe->setSource(m_player);
```

### 代码介绍
1. **信号与槽**
```cpp
    ///当QMediaPlayer的durationChanged信号发出时，Player类的durationChanged槽函数将被调用。durationChanged槽函数更新播放器的总时长。
    connect(m_player, &QMediaPlayer::durationChanged, this, &Player::durationChanged);
    ///当QMediaPlayer的positionChanged信号发出时，Player类的positionChanged槽函数将被调用。positionChanged槽函数通更新播放器的当前播放位置。
    connect(m_player, &QMediaPlayer::positionChanged, this, &Player::positionChanged);
    ///当移动进度条时，Player类的seek槽函数将被调用，从而改变媒体的播放位置。建立了媒体播放器进度条（QSlider）与播放器（Player）的连接
    connect(m_slider, &QSlider::sliderMoved, this, &Player::seek);
    //当点击进度条时，它将m_player的播放位置设置为点击处的值
    connect(m_slider, &ClickableSlider::clickedSlider, this, [&]()
            { m_player->setPosition(m_slider->value() * 1000); });
    //暂停/播放按钮
    connect(controlButton, &QPushButton::clicked, [&]()
            {
        if (m_player->state() == QMediaPlayer::PlayingState) {
            m_player->pause();
        } else {
            m_player->play();
        } });
    //退出按钮
    connect(eixtButton, &QPushButton::clicked, this, [&]()
            { close(); });
```
2. **当前播放时间文本**
- 在槽函数`positionChanged`中被调用。
```cpp
void Player::updateDurationInfo(qint64 currentInfo)
{
    QString tStr;
    if (currentInfo || m_duration)
    {
        QTime currentTime((currentInfo / 3600) % 60, (currentInfo / 60) % 60,
                          currentInfo % 60, (currentInfo * 1000) % 1000);
        QTime totalTime((m_duration / 3600) % 60, (m_duration / 60) % 60,
                        m_duration % 60, (m_duration * 1000) % 1000);
        QString format = "mm:ss";
        if (m_duration > 3600)
            format = "hh:mm:ss";
        tStr = currentTime.toString(format) + " / " + totalTime.toString(format);
    }
    m_labelDuration->setText(tStr);
}
```
3. **全屏**
- `QVideoWidget`有设置全屏的函数`setFullScreen`直接调用就行。
```cpp
void VideoWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    setFullScreen(!isFullScreen());
    event->accept();
}
```
4. **点击进度条位置播放**
- 自定义一个进度条`QSlider`，重写点击函数，主动触发自定义`clickedSlider`信号，链接上述的信号与槽。
```cpp
void ClickableSlider::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        // 根据点击位置计算新的值
        qreal value = minimum() + ((qreal)(maximum() - minimum()) * event->pos().x()) / width();
        setValue(value);
        emit clickedSlider();
        event->accept();
    }
    QSlider::mousePressEvent(event);
}
```
### 总结
- 知识理应共享，[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/pictureBrowser)在此。
- 这个案例功能相对简单，若是你想要更多功能，可以看看[Qt案例](https://gitee.com/shan-jie6/my-case/tree/master/QT/pictureBrowser)。
- qt6与qt5的API使用方式还是不一样的，这点需要注意。
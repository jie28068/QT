#include <QApplication>
#include <QMainWindow>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>

#include "videowidget.h"
#include "player.h"

const static QString dir = "D:/test/1.mp4";

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // // 主窗口
    // QWidget window;
    // // 创建媒体播放器和视频显示组件
    // QMediaPlayer *player = new QMediaPlayer;
    // QVideoWidget *videoWidget = new QVideoWidget;
    // player->setVideoOutput(videoWidget); // 设置视频输出
    // // 创建控件
    // QSlider *slider = new QSlider(Qt::Horizontal);
    // QPushButton *playButton = new QPushButton("Play/Pause");
    // // 设置滑动条的范围（以毫秒为单位）
    // slider->setRange(0, 0);
    // // 布局
    // QVBoxLayout *layout = new QVBoxLayout;
    // layout->addWidget(videoWidget);
    // layout->addWidget(slider);
    // layout->addWidget(playButton);
    // window.setLayout(layout);
    // // 信号和槽的连接
    // QObject::connect(playButton, &QPushButton::clicked, [&]()
    //                  {
    //     if (player->playbackState() == QMediaPlayer::PlayingState) {
    //         player->pause();
    //     } else {
    //         player->play();
    //     } });
    // // 更新滑动条位置
    // QObject::connect(player, &QMediaPlayer::positionChanged, [&](qint64 pos)
    //                  { slider->setValue(pos); });
    // // 设置滑动条的范围
    // QObject::connect(player, &QMediaPlayer::durationChanged, [&](qint64 dur)
    //                  { slider->setRange(0, dur); });
    // // 滑动条控制视频进度
    // QObject::connect(slider, &QSlider::sliderMoved, [&](int position)
    //                  { player->setPosition(position); });
    // QObject::connect(player, &QMediaPlayer::mediaStatusChanged, [&](QMediaPlayer::MediaStatus status) {
    // });
    // // 打开视频文件
    // QString fileName = QFileDialog::getOpenFileName(&window, "Open a Video");
    // player->setSource(QUrl::fromLocalFile(fileName));
    // window.resize(800, 600); // 设置窗口大小
    // window.show();           // 显示窗口
    // videoWidget->show();     // 显示视频组件
    // player->play();

    Player p;
    p.show();

    return a.exec();
}
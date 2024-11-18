#ifndef PLAYER_H
#define PLAYER_H

#include <QWidget>
#include <QMediaPlayer>
#include <QMediaPlaylist>
#include <QVideoWidget>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QSlider>

QT_BEGIN_NAMESPACE
class QLabel;
class QMediaPlayer;
class QPushButton;
class QSlider;
class QStatusBar;
class QVideoProbe;
class QVideoWidget;
class QAudioProbe;
QT_END_NAMESPACE

/// @brief 视图窗口 全屏
class VideoWidget : public QVideoWidget
{
    Q_OBJECT
public:
    VideoWidget(QWidget *parent = nullptr);

    void keyPressEvent(QKeyEvent *event);

    void mouseDoubleClickEvent(QMouseEvent *event);
};

/// @brief 滑动条   自定义位置
class ClickableSlider : public QSlider
{
    Q_OBJECT
public:
    ClickableSlider(Qt::Orientation orientation, QWidget *parent = nullptr);

Q_SIGNALS:
    void clickedSlider();

protected:
    void mousePressEvent(QMouseEvent *event) override;
};

class Player : public QWidget
{
    Q_OBJECT

public:
    explicit Player(QString dirname, QWidget *parent = nullptr);
    ~Player();
    void setCustomAudioRole(const QString &role);

private slots:
    void durationChanged(qint64 duration);
    void positionChanged(qint64 progress);
    void restartClicked();
    void seek(int seconds);

private:
    void updateDurationInfo(qint64 currentInfo);

    QMediaPlayer *m_player;
    QVideoWidget *m_videoWidget;
    ClickableSlider *m_slider;
    QLabel *m_labelDuration;
    QVideoProbe *m_videoProbe;

    qint64 m_duration;
};

#endif // PLAYER_H

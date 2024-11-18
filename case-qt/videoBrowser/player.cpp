#include "player.h"

#include <QMediaService>
#include <QMediaPlaylist>
#include <QVideoProbe>
#include <QAudioProbe>
#include <QMediaMetaData>
#include <QtWidgets>

VideoWidget::VideoWidget(QWidget *parent)
    : QVideoWidget(parent)
{
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    QPalette p = palette();
    p.setColor(QPalette::Window, Qt::black);
    setPalette(p);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

void VideoWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape && isFullScreen())
    {
        setFullScreen(false);
        event->accept();
    }
    else if (event->key() == Qt::Key_Enter && event->modifiers() & Qt::Key_Alt)
    {
        setFullScreen(!isFullScreen());
        event->accept();
    }
    else
    {
        QVideoWidget::keyPressEvent(event);
    }
}

void VideoWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    setFullScreen(!isFullScreen());
    event->accept();
}

ClickableSlider::ClickableSlider(Qt::Orientation orientation, QWidget *parent) : QSlider(orientation, parent) {}

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

Player::Player(QString dirname, QWidget *parent)
    : QWidget(parent)
{
    QVBoxLayout *centerLayout = new QVBoxLayout(this);
    QPushButton *eixtButton = new QPushButton(tr("exit"), this);
    QBoxLayout *exitlayout = new QHBoxLayout(this);
    exitlayout->addStretch(1);
    exitlayout->addWidget(eixtButton);

    m_player = new QMediaPlayer(this);
    m_player->setAudioRole(QAudio::VideoRole);
    m_player->setMuted(true); // 静音

    m_videoWidget = new VideoWidget(this);
    m_videoWidget->setMinimumSize(640, 480);
    m_player->setVideoOutput(m_videoWidget);

    m_slider = new ClickableSlider(Qt::Horizontal, this);
    m_slider->setRange(0, m_player->duration() / 1000);

    m_videoProbe = new QVideoProbe(this); // 关键显示问题 拦截和访问媒体播放器播放的视频帧
    m_videoProbe->setSource(m_player);

    QBoxLayout *displayLayout = new QHBoxLayout;
    displayLayout->addWidget(m_videoWidget, 2);

    QBoxLayout *controlLayout = new QHBoxLayout;
    QPushButton *controlButton = new QPushButton(tr("play/pause"), this);
    controlLayout->setContentsMargins(0, 0, 0, 0);

    controlLayout->addStretch(1);
    controlLayout->addWidget(controlButton);
    controlLayout->addStretch(1);

    QBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(displayLayout);
    m_labelDuration = new QLabel(this);
    QHBoxLayout *hLayout = new QHBoxLayout;
    hLayout->addWidget(m_slider);
    hLayout->addWidget(m_labelDuration);
    layout->addLayout(hLayout);
    layout->addLayout(controlLayout);

    centerLayout->addLayout(exitlayout);
    centerLayout->addLayout(layout);
    setLayout(centerLayout);

    // m_player->setSource(QUrl::fromLocalFile(fileName)); //QT6可以直接使用该接口，无需设置QVideoProbe
    m_player->setMedia(QUrl::fromLocalFile(dirname));
    m_player->play();

    connect(m_player, &QMediaPlayer::durationChanged, this, &Player::durationChanged);
    connect(m_player, &QMediaPlayer::positionChanged, this, &Player::positionChanged);
    connect(m_slider, &QSlider::sliderMoved, this, &Player::seek);
    connect(m_slider, &ClickableSlider::clickedSlider, this, [&]()
            { m_player->setPosition(m_slider->value() * 1000); });
    connect(controlButton, &QPushButton::clicked, [&]()
            {
        if (m_player->state() == QMediaPlayer::PlayingState) {
            m_player->pause();
        } else {
            m_player->play();
        } });

    connect(eixtButton, &QPushButton::clicked, this, [&]()
            { close(); });
}

Player::~Player()
{
}

void Player::durationChanged(qint64 duration)
{
    m_duration = duration / 1000;
    m_slider->setMaximum(m_duration);
}

void Player::positionChanged(qint64 progress)
{
    if (!m_slider->isSliderDown())
        m_slider->setValue(progress / 1000);
    updateDurationInfo(progress / 1000);
}

void Player::restartClicked()
{
    m_player->setPosition(0);
}

void Player::seek(int seconds)
{
    if (m_slider->isSliderDown())
        m_player->setPosition(seconds * 1000);
}

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

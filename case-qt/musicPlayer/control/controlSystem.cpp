#include "controlSystem.h"
#include "imageviewwindow.h"
#include "HistogramWidget.h"
#include "lyrics.h"
#include "graphicstext.h"
#include <QDir>
#include <QFileDialog>
#include <QAudioProbe>
#include <QTimer>
#include <QAction>
#include <QDockWidget>
#include <QToolBar>
#include <QMenu>

class CustomDockWidget : public QDockWidget
{
public:
    explicit CustomDockWidget(const QString &title, QWidget *parent = nullptr,
                              Qt::WindowFlags flags = Qt::WindowFlags()) : QDockWidget(title, parent, flags)
    {
        setFeatures(features() & ~QDockWidget::DockWidgetFloatable);
        setFeatures(features() & ~QDockWidget::DockWidgetClosable);
        setAllowedAreas(Qt::AllDockWidgetAreas);
    }
};

MainWindow::MainWindow(QWidget *parent)
{
    initUI();
    initConnect();

    lyric = new Lyrics();
}

MainWindow::~MainWindow()
{
}

void MainWindow::initUI()
{
    this->resize(1000, 800);
    // 上边
    QToolBar *toolBar = new QToolBar("菜单栏", this);
    QAction *menuAction = new QAction("歌曲操作", this);
    QMenu *menu = new QMenu(this);
    addAction = menu->addAction("添加歌曲");
    deleteAction = menu->addAction("删除歌曲");
    clsAction = menu->addAction("清空歌曲");
    menuAction->setMenu(menu);
    toolBar->addAction(menuAction);

    imageAction = new QAction("背景图片", this);
    toolBar->addAction(imageAction);
    this->addToolBar(Qt::TopToolBarArea, toolBar);
    // 左边
    auto leftDockWidget = new CustomDockWidget("播放列表", this);
    auto *leftwidget = new QWidget(leftDockWidget);
    auto verticalLayout = new QVBoxLayout(leftwidget);
    listWidget = new QListWidget(leftwidget);
    verticalLayout->addWidget(listWidget);
    leftwidget->setLayout(verticalLayout);
    leftwidget->setBaseSize(width() / 5, height());
    leftDockWidget->setWidget(leftwidget);
    this->addDockWidget(Qt::LeftDockWidgetArea, leftDockWidget);

    // 中间
    imageViewWindow = new ImageViewWindow(this);
    setCentralWidget(imageViewWindow);

    // 下边
    auto bomDockWidget = new CustomDockWidget("操作台", this);
    auto bomWidget = new QWidget(bomDockWidget);
    auto *verticalLayout_2 = new QVBoxLayout(bomWidget);
    auto horizontalLayout = new QHBoxLayout(bomWidget);
    horizontalLayout->setSpacing(6);
    btnPlay = new QPushButton(bomWidget);
    btnPlay->setText("播放");
    horizontalLayout->addWidget(btnPlay);

    btnPause = new QPushButton(bomWidget);
    btnPause->setText("暂停");
    horizontalLayout->addWidget(btnPause);

    btnStop = new QPushButton(bomWidget);
    btnStop->setText("停止");
    horizontalLayout->addWidget(btnStop);

    btnPrevious = new QPushButton(bomWidget);
    btnPrevious->setText("上一首");
    horizontalLayout->addWidget(btnPrevious);

    btnNext = new QPushButton(bomWidget);
    btnNext->setText("下一首");
    horizontalLayout->addWidget(btnNext);

    horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    horizontalLayout->addItem(horizontalSpacer);

    btnSound = new QPushButton(bomWidget);
    btnSound->setText("音量");
    btnSound->setFlat(true);
    horizontalLayout->addWidget(btnSound);

    sliderVolumn = new QSlider(bomWidget);
    sliderVolumn->setMaximum(100);
    sliderVolumn->setValue(100);
    sliderVolumn->setOrientation(Qt::Horizontal);
    horizontalLayout->addWidget(sliderVolumn);
    verticalLayout_2->addLayout(horizontalLayout);
    line = new QFrame(bomWidget);
    line->setFrameShadow(QFrame::Plain);
    line->setFrameShape(QFrame::HLine);
    verticalLayout_2->addWidget(line);
    auto horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setSpacing(9);

    LabCurMedia = new QLabel(bomWidget);
    LabCurMedia->setText("无曲目");

    LabCurMedia->setMinimumSize(QSize(100, 0));

    horizontalLayout_2->addWidget(LabCurMedia);

    sliderPosition = new QSlider(bomWidget);
    sliderPosition->setTracking(false);
    sliderPosition->setOrientation(Qt::Horizontal);

    horizontalLayout_2->addWidget(sliderPosition);

    LabRatio = new QLabel(bomWidget);
    LabRatio->setText("00:00/00:00");
    LabRatio->setMinimumSize(QSize(80, 0));

    horizontalLayout_2->addWidget(LabRatio);
    verticalLayout_2->addLayout(horizontalLayout_2);
    bomWidget->setLayout(verticalLayout_2);
    bomDockWidget->setWidget(bomWidget);
    this->addDockWidget(Qt::BottomDockWidgetArea, bomDockWidget);

    // 右边
    auto rightDockWidget = new CustomDockWidget("音频图", this);
    player = new QMediaPlayer(this);
    playlist = new QMediaPlaylist(this);
    playlist->setPlaybackMode(QMediaPlaylist::Loop); // 循环模式
    player->setPlaylist(playlist);

    auto m_audioHistogram = new HistogramWidget(rightDockWidget);
    auto probe = new QAudioProbe(this);
    connect(probe, &QAudioProbe::audioBufferProbed, m_audioHistogram, &HistogramWidget::processBuffer);
    probe->setSource(player);
    rightDockWidget->setWidget(m_audioHistogram);
    this->addDockWidget(Qt::RightDockWidgetArea, rightDockWidget);
}

void MainWindow::initConnect()
{

    connect(player, SIGNAL(stateChanged(QMediaPlayer::State)),
            this, SLOT(onStateChanged(QMediaPlayer::State)));

    connect(player, SIGNAL(positionChanged(qint64)),
            this, SLOT(onPositionChanged(qint64)));

    connect(player, SIGNAL(durationChanged(qint64)),
            this, SLOT(onDurationChanged(qint64)));

    connect(playlist, SIGNAL(currentIndexChanged(int)),
            this, SLOT(onPlaylistChanged(int)));

    connect(addAction, &QAction::triggered, this, &MainWindow::on_btnAdd_clicked);
    connect(deleteAction, &QAction::triggered, this, &MainWindow::on_btnRemove_clicked);
    connect(clsAction, &QAction::triggered, this, &MainWindow::on_btnClear_clicked);
    connect(imageAction, &QAction::triggered, this, &MainWindow::on_imageAction_triggered);

    connect(btnPlay, &QPushButton::clicked, this, &MainWindow::on_btnPlay_clicked);
    connect(btnPause, &QPushButton::clicked, this, &MainWindow::on_btnPause_clicked);
    connect(btnStop, &QPushButton::clicked, this, &MainWindow::on_btnStop_clicked);
    connect(btnPrevious, &QPushButton::clicked, this, &MainWindow::on_btnPrevious_clicked);
    connect(btnNext, &QPushButton::clicked, this, &MainWindow::on_btnNext_clicked);
    connect(btnSound, &QPushButton::clicked, this, &MainWindow::on_btnSound_clicked);
    connect(listWidget, &QAbstractItemView::doubleClicked, this, &MainWindow::on_listWidget_doubleClicked);
    connect(sliderVolumn, &QSlider::valueChanged, this, &MainWindow::on_sliderVolumn_valueChanged);
    connect(sliderPosition, &QSlider::valueChanged, this, &MainWindow::on_sliderPosition_valueChanged);
}

void MainWindow::updatePositionTime(qint64 position)
{
    // int pos = position / 10;
    // qDebug() << "外侧pos:" << pos;
    // auto lrcMap = lyric->getListLyricsMap();
    // QMap<int, QString>::iterator iter = lrcMap.begin();
    // while (iter != lrcMap.end())
    // {
    //     if (pos - 50 <= iter.key() && pos + 50 >= iter.key())
    //     {
    //         qDebug() << "pos:" << pos << "iter.key():" << iter.key() << "iter.value():" << iter.value();
    //         int j = 0;
    //         if (iter != lrcMap.begin())
    //         {
    //             iter--;
    //             imageViewWindow->getItems()[0]->setStr(iter.value());
    //             j++;
    //         }
    //         if (iter != lrcMap.begin())
    //         {
    //             iter--;
    //             imageViewWindow->getItems()[1]->setStr(iter.value());
    //             j++;
    //         }
    //         if (iter != lrcMap.begin())
    //         {
    //             iter--;
    //             imageViewWindow->getItems()[2]->setStr(iter.value());
    //             j++;
    //         }

    //         imageViewWindow->getItems()[3]->setStr(iter.value());
    //         iter++;

    //         if (iter != lrcMap.end())
    //         {
    //             imageViewWindow->getItems()[4]->setStr(iter.value());
    //         }
    //         iter++;
    //         if (iter != lrcMap.end())
    //         {
    //             imageViewWindow->getItems()[5]->setStr(iter.value());
    //         }
    //         iter++;
    //         if (iter != lrcMap.end())
    //         {
    //             imageViewWindow->getItems()[6]->setStr(iter.value());
    //         }
    //         iter++;
    //     }
    //     iter++;
    // }
}

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

    LabCurMedia->setText(currntStr);

    imageViewWindow->textChanged(displayLyrics);
}

void MainWindow::updateLrc()
{
    int currentRow = -1;
    for (int i = 0; i < playlist->mediaCount(); ++i)
    {
        if (playlist->media(i) == playlist->currentMedia())
        {
            currentRow = i;
            break;
        }
    }
    auto item = listWidget->item(currentRow);
    if (item)
    {
        QString text = item->text();
        lyric->readLyricsFile(lrcMap[text]);
    }
}

void MainWindow::onStateChanged(QMediaPlayer::State state)
{
    btnPlay->setEnabled(!(state == QMediaPlayer::PlayingState));
    btnPause->setEnabled(state == QMediaPlayer::PlayingState);
    btnStop->setEnabled(state == QMediaPlayer::PlayingState);
}
void MainWindow::onPlaylistChanged(int position)
{
    listWidget->setCurrentRow(position);
    QListWidgetItem *item = listWidget->currentItem();
    if (item)
    {
        updateLrc();
        LabCurMedia->setText(item->text());
    }
}

void MainWindow::onDurationChanged(qint64 duration)
{
    sliderPosition->setMaximum(duration);

    int secs = duration / 1000; // 秒
    int mins = secs / 60;       // 分钟
    secs = secs % 60;           // 余数秒
    durationTime = QString::asprintf("%d:%d", mins, secs);
    LabRatio->setText(positionTime + "/" + durationTime);
}

void MainWindow::onPositionChanged(qint64 position)
{
    if (sliderPosition->isSliderDown())
        return;

    sliderPosition->setSliderPosition(position); //

    int secs = position / 1000; // 秒
    int mins = secs / 60;       // 分钟
    secs = secs % 60;           // 余数秒
    positionTime = QString::asprintf("%d:%d", mins, secs);
    LabRatio->setText(positionTime + "/" + durationTime);

    updateTextTime(position);
}

void MainWindow::on_btnAdd_clicked()
{
    QString curPath = QDir::homePath();
    QString dlgTitle = "选择音频文件";
    QString filter = "音频文件(*.mp3 *.wav *.wma);;mp3文件(*.mp3);;wav文件(*.wav);;wma文件(*.wma);;所有文件(*.*)";
    QString file = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);

    if (file.count() < 1)
        return;

    QString aFile = file;
    playlist->addMedia(QUrl::fromLocalFile(aFile)); // 添加文件

    QFileInfo fileInfo(aFile);
    auto name = fileInfo.baseName();
    listWidget->addItem(name); // 添加到界面文件列表
    auto path = fileInfo.path();
    QString lrc = QString("%1/%2.lrc").arg(path).arg(name); // 歌词文件
    lrcMap.insert(name, lrc);
}

void MainWindow::on_btnPlay_clicked()
{
    if (playlist->currentIndex() < 0)
        playlist->setCurrentIndex(0);
    player->play();
    updateLrc();
}

void MainWindow::on_btnPause_clicked()
{
    player->pause();
}

void MainWindow::on_btnStop_clicked()
{
    player->stop();
}

void MainWindow::on_listWidget_doubleClicked(const QModelIndex &index)
{
    int rowNo = index.row();
    QListWidgetItem *item = listWidget->item(rowNo);
    if (item)
    {
        QString text = item->text();
        lyric->readLyricsFile(lrcMap[text]);
    }
    playlist->setCurrentIndex(rowNo);
    player->play();
}

void MainWindow::on_btnClear_clicked()
{
    playlist->clear();
    listWidget->clear();
    player->stop();
    lrcMap.clear();
}

void MainWindow::on_sliderVolumn_valueChanged(int value)
{
    player->setVolume(value);
}

void MainWindow::on_btnSound_clicked()
{
    bool mute = player->isMuted();
    player->setMuted(!mute);
    if (mute)
        btnSound->setText("音量(开)");
    else
        btnSound->setText("音量(关)");
}

void MainWindow::on_sliderPosition_valueChanged(int value)
{
    player->setPosition(value);
}

void MainWindow::on_btnRemove_clicked()
{
    int pos = listWidget->currentRow();
    QListWidgetItem *item = listWidget->takeItem(pos);
    lrcMap.remove(item->text());
    delete item; // 从listWidget里删除

    if (playlist->currentIndex() == pos) // 是当前播放的曲目
    {
        int nextPos = 0;
        if (pos >= 1)
            nextPos = pos - 1;

        playlist->removeMedia(pos); // 从播放列表里移除
        if (listWidget->count() > 0)
        {
            playlist->setCurrentIndex(nextPos);
            onPlaylistChanged(nextPos);
        }
        else
        {
            player->stop();
            LabCurMedia->setText("无曲目");
        }
    }
    else
        playlist->removeMedia(pos);
}

void MainWindow::on_btnPrevious_clicked()
{
    playlist->previous();

    updateLrc();
}

void MainWindow::on_btnNext_clicked()
{
    playlist->next();

    updateLrc();
}

void MainWindow::on_imageAction_triggered()
{
    QString curPath = QDir::homePath();
    QString dlgTitle = "选择图片";
    QString filter = "JPG(*.jpg);;PNG(*.png)";
    auto str = QFileDialog::getOpenFileName(this, dlgTitle, curPath, filter);
    imageViewWindow->setImage(str);
}

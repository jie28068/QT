#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include <QMainWindow>
#include <QMediaPlayer>
#include <QMediaPlaylist>
#include <QLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QListWidget>
#include <QSlider>
#include <QLabel>
#include <QQueue>

class ImageViewWindow;
class Lyrics;
class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    QMediaPlayer *player;     // 播放器
    QMediaPlaylist *playlist; // 播放列表
    QString durationTime;     // 总长度
    QString positionTime;     // 当前播放到位置
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void initUI();
    void initConnect();
    void updatePositionTime(qint64 position);
    /// @brief 同步歌词
    /// @param duration
    void updateTextTime(qint64 duration);
    /// @brief 更新歌词
    void updateLrc();

private slots:
    // 播放器状态变化，更新按钮状态
    void onStateChanged(QMediaPlayer::State state);
    // 播放列表变化,更新当前播放文件名显示
    void onPlaylistChanged(int position);
    // 文件时长变化，更新进度显示
    void onDurationChanged(qint64 duration);
    // 当前文件播放位置变化，更新进度显示
    void onPositionChanged(qint64 position);
    // 添加文件
    void on_btnAdd_clicked();
    // 播放
    void on_btnPlay_clicked();
    // 暂停播放
    void on_btnPause_clicked();
    // 停止播放
    void on_btnStop_clicked();
    // 双击时切换播放文件
    void on_listWidget_doubleClicked(const QModelIndex &index);
    // 清空列表
    void on_btnClear_clicked();
    // 调整音量
    void on_sliderVolumn_valueChanged(int value);
    // 静音控制
    void on_btnSound_clicked();
    // 文件进度调控
    void on_sliderPosition_valueChanged(int value);
    // 移除一个文件
    void on_btnRemove_clicked();
    // 前一文件
    void on_btnPrevious_clicked();
    // 下一文件
    void on_btnNext_clicked();
    // 修改背景图片
    void on_imageAction_triggered();

private:
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QAction *addAction;
    QAction *deleteAction;
    QAction *clsAction;
    QAction *imageAction;
    QListWidget *listWidget;
    QPushButton *btnPlay;
    QPushButton *btnPause;
    QPushButton *btnStop;
    QPushButton *btnPrevious;
    QPushButton *btnNext;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnSound;
    QSlider *sliderVolumn;
    QFrame *line;
    QLabel *LabCurMedia;
    QSlider *sliderPosition;
    QLabel *LabRatio;
    Lyrics *lyric;
    ImageViewWindow *imageViewWindow;
    QMap<QString, QString> lrcMap;
};

#endif
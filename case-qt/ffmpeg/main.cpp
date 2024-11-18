#include <QApplication>
#include <QVideoWidget>
#include <QMediaPlayer>
#include <QMediaPlaylist>
#include <QUrl>
#include <QWidget>

extern "C"
{
#include <libavutil/log.h>
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    av_log_set_level(AV_LOG_DEBUG);
    av_log(NULL, AV_LOG_INFO, "Hello World!\n");

    return app.exec();
}
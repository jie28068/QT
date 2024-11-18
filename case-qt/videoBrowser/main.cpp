#include <QApplication>
#include <QMainWindow>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>

#include "player.h"

const static QString dir = "D:/test/1.mp4";

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Player p(dir);
    p.show();

    return a.exec();
}
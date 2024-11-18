#include "../heard/mainwindow.h"
#include <QApplication>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MyMainWindow w;
    w.setWindowTitle("点睛之笔");
    w.resize(640, 480);
    w.show();
    return a.exec();
}
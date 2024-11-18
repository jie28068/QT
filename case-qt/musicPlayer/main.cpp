#pragma execution_character_set("utf-8")

#include "imageviewwindow.h"
#include "controlSystem.h"
#include <QApplication>
#include <QTextCodec>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}

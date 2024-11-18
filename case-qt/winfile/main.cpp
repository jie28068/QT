#include "mainWindows.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    FileExplorer explorer;
    explorer.show();

    return a.exec();
}
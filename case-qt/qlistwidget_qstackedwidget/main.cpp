#include "tablewidget.h"
#include "stackedwidget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindowS widget;
    widget.show();

    return app.exec();
}
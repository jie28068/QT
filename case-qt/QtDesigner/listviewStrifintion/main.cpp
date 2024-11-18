#include <QApplication>
#include <QWidget>

#include "controlTool.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    ControlTools *w = new ControlTools();
    w->resize(800, 600);
    w->addPage("Page 1");
    w->addPage("Page 2");
    w->addPage("Page 3");
    w->show();

    return app.exec();
}
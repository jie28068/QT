#include <QtWidgets>

#include "mianWindows.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    Scene *scene = new Scene;
    scene->refreshMouse(7);

    MianWindows mainwindows(scene);
    mainwindows.show();

    QTimer timer;
    QObject::connect(&timer, SIGNAL(timeout()), scene, SLOT(advance()));
    timer.start(1000 / 33);

    return app.exec();
}
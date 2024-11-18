#include "widget/mainwindow.h"

#include <QApplication>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    MainWindow window;
    window.setWindowIcon(QIcon(":images/m"));
    // 关于QStringLiteral功能：编译时进行处理并能够提高字符串的性能和安全性。生成一个 QStringData 对象。
    window.loadImage(QStringLiteral(":/images/m"));
    window.show();
    return app.exec();
}
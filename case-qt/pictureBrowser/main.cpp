#include "ImageBrowser.h"

// const static QString dirname = "D:/other/image/test/";
const static QString dirname = "D:/other/image/image/";

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    ImageBrowser *browser = new ImageBrowser(dirname);
    browser->show();

    return app.exec();
}
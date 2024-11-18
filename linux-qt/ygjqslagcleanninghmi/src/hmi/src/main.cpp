#include "mainWindow.h"
#include <QApplication>
#include <QIcon>

#include "glog/logging.h"
#include <sys/stat.h>
#include <sys/types.h>

void ErrorCallback(const char *data, int size)
{
    std::string ss(data, size - 1);
    LOG(ERROR) << ss;
}

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter(ErrorCallback);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbuflevel = google::INFO;
    mkdir("./logs", S_IRWXU | S_IRWXG | S_IRWXO);
    FLAGS_log_dir = "./logs";

    QApplication a(argc, argv);
    QApplication::setWindowIcon(QIcon(":/resource/icon.jpg"));
    MainWindow w(argc, argv);

    w.showFullScreen();

    return a.exec();
}

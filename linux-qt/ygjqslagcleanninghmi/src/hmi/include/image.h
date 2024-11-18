#ifndef IMAGE_H
#define IMAGE_H

#include <QWidget>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include "qtnode.h"

namespace Ui {
class Image;
}

class Image : public QWidget
{
    Q_OBJECT

public:
    explicit Image(QtNode *node_ptr, QWidget *parent = nullptr);
    ~Image();
    // void displayMat(cv::Mat image);

private:
    Ui::Image *ui;
    QtNode *qt_node_ = nullptr;
    cv::Mat hole1_image;
    void init_image();
};

#endif // IMAGE_H
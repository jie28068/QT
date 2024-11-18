#include "image.h"
#include "ui_image.h"

Image::Image(QtNode *node_ptr, QWidget *parent) :
    QWidget(parent),
    qt_node_(node_ptr),
    ui(new Ui::Image)
{
    ui->setupUi(this);
    init_image();
}

Image::~Image()
{
    delete ui;
}

void Image::init_image() {
    qDebug()<<"进入图片";
    // 加载图片
    cv::Mat img = cv::imread("/home/ubuntu/test/hmi_qt_template/src/hmi/resource/主界面.png");
    // 检查图片是否加载成功
    if(img.empty()) {
        qDebug()<<"图片加载失败";
        return;
    }
    qDebug()<<"图片加载成功";
    cv::Mat new_img;
    cvtColor(img, new_img, CV_BGR2RGB);
    // 将cv::Mat转换为QImage
    QImage qImage = QImage((const unsigned char*)(new_img.data),
                           new_img.cols,
                           new_img.rows,
                           new_img.step,
                           QImage::Format_RGB888);
 
    // 将QImage转换为QPixmap
    QPixmap pixmap = QPixmap::fromImage(qImage);
    // 图片自适应控件大小
    ui->label_image_hole_1->setScaledContents(true);
    // 创建一个标签来显示图片
    ui->label_image_hole_1->setPixmap(pixmap);
    ui->label_image_hole_1->show();    
}
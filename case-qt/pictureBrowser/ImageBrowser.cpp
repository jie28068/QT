#include "ImageBrowser.h"

ImageBrowser::ImageBrowser(const QString &strdir, QWidget *parent)
    : QDialog(parent), dirname(strdir)
{
    // 主窗口
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    // 图片显示区域
    imageLabel = new ShowLabel(this);
    imageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    imageLabel->setMinimumSize(QSize(100, 100));
    imageLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(imageLabel, 3);

    // 滚动区域内容窗口部件
    QWidget *scrollAreaContent = new QWidget();
    QHBoxLayout *thumbnailsLayout = new QHBoxLayout(scrollAreaContent);
    scrollAreaContent->setLayout(thumbnailsLayout);
    // 滚动区域
    scrollArea = new QScrollArea(this);
    scrollArea->setWidget(scrollAreaContent);
    scrollArea->setWidgetResizable(true);
    scrollArea->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    scrollArea->setMinimumSize(QSize(100, 100));
    mainLayout->addWidget(scrollArea, 1);
    // 设置主窗口的布局
    this->setLayout(mainLayout);

    QDir dir(dirname);
    QStringList images = dir.entryList(QStringList() << "*.jpg" << "*.jpeg" << "*.png" << "*.bmp" << "*.svg" << "*.gif" << "*.tiff" << "*.tif", QDir::Files);
    for (QString filename : images)
    {
        QPixmap pixmap(dirname + filename);
        ClickableLabel *label = new ClickableLabel(this);
        label->setObjectName(filename);
        label->setPixmap(pixmap.scaled(100, 100, Qt::KeepAspectRatio));
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        thumbnailsLayout->addWidget(label);
        labels.append(label);
        connect(label, &ClickableLabel::clicked, this, &ImageBrowser::thumbnailClicked);
    }

    if (!images.isEmpty())
    {
        currentImageIndex = 0;
        showImage(images.first());
    }
}

void ImageBrowser::showImage(const QString &filename)
{
    QPixmap pixmap(dirname + filename);
    imageLabel->resert();
    imageLabel->setPixmap(pixmap.scaled(400, 400, Qt::KeepAspectRatio));

    // 动画效果，使选中的缩略图居中
    QScrollBar *scrollBar = scrollArea->horizontalScrollBar();
    int targetValue = labels[currentImageIndex]->geometry().center().x() - (scrollArea->width() / 2);
    // 创建动画并执行
    QPropertyAnimation *animation = new QPropertyAnimation(scrollBar, "value");
    animation->setDuration(1000);
    animation->setStartValue(scrollBar->value());
    animation->setEndValue(targetValue);
    animation->start();

    // 移除之前选中的缩略图的边框
    for (auto label : labels)
    {
        label->setStyleSheet("");
    }
    // 为当前选中的缩略图添加边框
    auto currentThumbnail = labels[currentImageIndex];
    currentThumbnail->setStyleSheet("border: 4px solid lightskyblue;");
}

void ImageBrowser::wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
    {
        showPreviousImage();
    }
    else if (event->delta() < 0)
    {
        showNextImage();
    }
    event->accept();
}

void ImageBrowser::showNextImage()
{
    if (currentImageIndex < labels.size() - 1)
    {
        currentImageIndex += 1;
        showImage(labels[currentImageIndex]->objectName());
    }
}

void ImageBrowser::thumbnailClicked()
{
    ClickableLabel *clickedLabel = qobject_cast<ClickableLabel *>(sender());
    if (clickedLabel)
    {
        // 更新currentImageIndex以反映被点击的标签
        currentImageIndex = labels.indexOf(clickedLabel);
        // 显示被点击的图片
        showImage(clickedLabel->objectName());
    }
}

void ImageBrowser::showPreviousImage()
{
    if (currentImageIndex > 0)
    {
        currentImageIndex -= 1;
        showImage(labels[currentImageIndex]->objectName());
    }
}

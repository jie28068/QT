#ifndef IMAGEBROWSER_H
#define IMAGEBROWSER_H

#include <QApplication>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QDir>
#include <QScrollBar>
#include <QScrollArea>
#include <QPropertyAnimation>
#include <QPixmap>
#include <QMouseEvent>
#include <QDialog>

#include "MyLabel.h"

class ImageBrowser : public QDialog
{
    Q_OBJECT
public:
    ImageBrowser(const QString &dir = QString(), QWidget *parent = nullptr);

private:
    void showImage(const QString &filename);

protected:
    void wheelEvent(QWheelEvent *event);
private slots:
    void showPreviousImage();

    void showNextImage();

    void thumbnailClicked();

private:
    ShowLabel *imageLabel;
    QScrollArea *scrollArea;
    QHBoxLayout *thumbnailsLayout;
    QList<ClickableLabel *> labels;
    int currentImageIndex;
    QString dirname;
};

#endif
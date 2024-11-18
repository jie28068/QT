#if 1
#include "mwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}
#else
#include <QApplication>
#include <QProgressBar>
#include <QMouseEvent>
#include <QPainter>
#include <QHBoxLayout>
class CustomProgressBar : public QProgressBar
{
public:
    CustomProgressBar(QWidget *parent = nullptr) : QProgressBar(parent), dragging(false) {}

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            int pos = event->x();
            int width = this->width();
            int progress = (int)(((double)pos / width) * this->maximum());
            this->setValue(progress);
            dragging = true;
        }
        QProgressBar::mousePressEvent(event);
    }
    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (dragging)
        {
            int pos = event->x();
            int width = this->width();
            int progress = (int)(((double)pos / width) * this->maximum());
            this->setValue(progress);
        }
        QProgressBar::mouseMoveEvent(event);
    }
    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            dragging = false;
        }
        QProgressBar::mouseReleaseEvent(event);
    }
    void paintEvent(QPaintEvent *event) override
    {
        QProgressBar::paintEvent(event); // 调用基类的paintEvent()以绘制基本的进度条
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        // 绘制圆形指示器
        int width = this->width();
        int height = this->height();
        int progress = this->value();
        int max = this->maximum();
        double ratio = double(progress) / double(max);
        int circleWidth = width * ratio;
        int circleHeight = height / 2;
        painter.setBrush(QColor("#3498db"));
        painter.drawEllipse(circleWidth - (height / 2), circleHeight - (height / 2), height, height);
    }

private:
    bool dragging;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget *widget = new QWidget;
    CustomProgressBar progressBar;
    progressBar.resize(250, 50);
    progressBar.setMinimum(0);
    progressBar.setMaximum(100);
    progressBar.setValue(50);
    QHBoxLayout *layout = new QHBoxLayout(widget);
    layout->addWidget(&progressBar);
    widget->setLayout(layout);
    widget->resize(250, 100);
    widget->show();
    return app.exec();
}
#endif
// #include <QApplication>

// #include <QPainter>
// #include <QPixmap>
// #include <QRegion>
// #include <QBitmap>
// #include <QWidget>
// class MaskedWidget : public QWidget
// {
// public:
//     MaskedWidget(QWidget *parent = nullptr) : QWidget(parent)
//     {
//         setMinimumSize(300, 200);
//     }
// };

// int main(int argc, char *argv[])
// {
//     QApplication a(argc, argv);

//     MaskedWidget *m = new MaskedWidget;
//     m->show();

//     // 假设有个名为myWidget的QWidget实例
//     QWidget *myWidget = m;

//     // 创建一个与Widget相同大小的QPixmap
//     QPixmap pixmap(myWidget->size());
//     pixmap.fill(Qt::black);

//     // 创建并初始化一个QPainter用于绘制
//     QPainter painter(&pixmap);
//     painter.setPen(Qt::NoPen);
//     painter.setBrush(Qt::white);

//     // 绘制圆形区域
//     int radius = std::min(pixmap.width(), pixmap.height()) / 2;
//     painter.drawEllipse(pixmap.rect().center(), radius, radius);

//     // 清理并应用遮罩
//     painter.end();
//     QBitmap mask = QBitmap::fromImage(pixmap.toImage().createHeuristicMask());
//     myWidget->setMask(mask);

//     myWidget->show();

//     return a.exec();
// }

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QPixmap>
#include <QBitmap>
#include <QLabel>
#include <QVBoxLayout>

QString MaskRect = "1";
QString MaskRound = "2";
QString MaskOctangle = "3";

class MaskedWidget : public QWidget
{
public:
    MaskedWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
    }

    void setCameraMask(QString mask)
    {
        QPixmap bitmap(size());
        bitmap.fill(QColor(0, 0, 0, 0));
        QPainter painter(&bitmap);
        painter.setBrush(QColor(255, 0, 0, 255));

        if (mask == MaskRect) // 矩形
        {
            painter.drawRect(bitmap.rect());
        }
        else if (mask == MaskRound) // 圆形
        {
            int radius = std::min(bitmap.width(), bitmap.height()) / 2;
            painter.drawEllipse(bitmap.rect().center(), radius, radius);
        }
        else if (mask == MaskOctangle) // 八边形
        {
            int side = std::min(bitmap.width(), bitmap.height());
            QPolygon octagon;
            for (int i = 0; i < 8; ++i)
            {
                qreal angle = 3.14159265358979323846 * i / 4.0; // 45度
                QPoint point(side / 2 * cos(angle), side / 2 * sin(angle));
                octagon.append(point + bitmap.rect().center());
            }
            painter.drawPolygon(octagon);
        }
        else
        {
            setMask(QPixmap());
            return;
        }
        painter.end();
        QBitmap maskdir = QBitmap::fromImage(bitmap.toImage().createHeuristicMask());
        setMask(maskdir);
    }
};

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MaskedWidget *m = new MaskedWidget;
    m->show();
    m->setCameraMask("3");

    return a.exec();
}

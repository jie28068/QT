#include "mlabel.h"
#include <QMouseEvent>
#include <QMenu>
#include <QPixmap>

MyLable::MyLable(QWidget *parent)
    : QLabel(parent),
      m_scaleValue(1.0),
      m_mousePoint(0, 0),
      m_drawPoint(0, 0),
      m_rectPixmap(0, 0, 0, 0),
      m_isMousePress(0),
      SCALE_MAX_VALUE(10.0),
      SCALE_MIN_VALUE(0.5)
{
}

void MyLable::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::black);
    painter.setPen(pen);
    const QPixmap *pixmap = this->pixmap();
    if (!pixmap || pixmap->isNull())
    {
        painter.drawRect(1, 1, LABLE_WIDTH - 2, LABLE_HEIGHT - 2);
        painter.drawText(LABLE_WIDTH / 4, LABLE_HEIGHT / 2, "未设置图片");
        return;
    }
    double width = this->width() * m_scaleValue;
    double height = this->height() * m_scaleValue;
    QPixmap scalePixmap =
        this->pixmap()->scaled(width, height, Qt::KeepAspectRatio, Qt::SmoothTransformation); // 饱满缩放
    m_rectPixmap = QRect(m_drawPoint.x(), m_drawPoint.y(), width, height);                    // 图片区域
    painter.drawPixmap(m_rectPixmap, scalePixmap);
}

void MyLable::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isMousePress)
    {
        int x = event->pos().x() - m_mousePoint.x();
        int y = event->pos().y() - m_mousePoint.y();
        m_mousePoint = event->pos();
        m_drawPoint = QPointF(m_drawPoint.x() + x, m_drawPoint.y() + y);
        update();
    }
}

void MyLable::mousePressEvent(QMouseEvent *event)
{
    if (event->type() == QEvent::MouseButtonDblClick)
    {
        if (event->button() == Qt::LeftButton)
        {
            return;
        }
    }
    if (event->button() == Qt::LeftButton)
    {
        m_isMousePress = true;
        m_mousePoint = event->pos();
    }
}

void MyLable::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        m_drawPoint = QPointF(0, 0);
        m_scaleValue = 1.0;
        update();
    }
    if (event->button() == Qt::LeftButton)
        m_isMousePress = false;
}

void MyLable::wheelEvent(QWheelEvent *event)
{
    changeWheelValue(event->pos(), event->delta());
    event->accept();
}

void MyLable::resizeEvent(QResizeEvent *event)
{
    m_drawPoint = QPointF(0, 0);
    m_scaleValue = 1.0;
    update();
}

void MyLable::changeWheelValue(QPoint event, int numSteps)
{
    double oldScale = m_scaleValue;
    if (numSteps > 0)
    {
        m_scaleValue *= 1.1;
    }
    else
    {
        m_scaleValue *= 0.9;
    }
    if (m_scaleValue > (SCALE_MAX_VALUE))
    {
        m_scaleValue = SCALE_MAX_VALUE;
    }
    if (m_scaleValue < (SCALE_MIN_VALUE))
    {
        m_scaleValue = SCALE_MIN_VALUE;
    }

    if (m_rectPixmap.contains(event))
    {
        double x = m_drawPoint.x() - (event.x() - m_drawPoint.x()) / m_rectPixmap.width() * (this->width() * (m_scaleValue - oldScale));
        double y = m_drawPoint.y() - (event.y() - m_drawPoint.y()) / m_rectPixmap.height() * (this->height() * (m_scaleValue - oldScale));
        m_drawPoint = QPointF(x, y);
    }
    else
    {
        double x = m_drawPoint.x() - (this->width() * (m_scaleValue - oldScale)) / 2;
        double y = m_drawPoint.y() - (this->height() * (m_scaleValue - oldScale)) / 2;
        m_drawPoint = QPointF(x, y);
    }
    update();
}

bool MyLable::event(QEvent *e)
{
    if (e->type() == QEvent::ToolTip)
    {
        QToolTip::showText(mapToGlobal(QPoint(0, 0)), "图片");
        return true;
    }
    return QLabel::event(e);
}
void MyLable::contextMenuEvent(QContextMenuEvent *ev)
{
    QMenu *menu = new QMenu(this);
    QAction *input = new QAction("导入图片", this);  // 导入图片
    QAction *output = new QAction("导出图片", this); // 导出图片
    QAction *delect = new QAction("删除图片", this); // 删除图片
    menu->addAction(input);
    menu->addAction(output);
    menu->addAction(delect);
    QAction *act = menu->exec(QCursor::pos());
    if (act && act->text() == "导入图片")
    {
        // auto filePath = QFileDialog::getOpenFileName(&QWidget(), "", "", "PNG(*.png)");
        // if (!filePath.isEmpty())
        // {
        //     QImage image;
        //     QFile file(filePath);
        //     file.open(QFile::ReadOnly);
        //     image.loadFromData(file.readAll());
        //     setPixmap(QPixmap::fromImage(image));
        //     show();
        // }
        emit changePixmp();
    }
    else if (act && act->text() == "导出图片")
    {

        auto fileName = QFileDialog::getSaveFileName(&QWidget(), "导出图片", QString(), "读出(*.png *.jpg)");
        QFile file(fileName);
        pixmap()->save(fileName);
        file.close();
    }
    else if (act && act->text() == "删除图片")
    {
        // setPixmap(QPixmap());
        // update();
        emit delPixmp();
    }
}
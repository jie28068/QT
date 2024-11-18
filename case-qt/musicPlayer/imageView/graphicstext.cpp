#include "graphicstext.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QCursor>
#include <QDebug>

GraphicsText::GraphicsText() : QGraphicsObject()
{
	m_font.setFamily("Microsoft YaHei");
	m_font.setPixelSize(12);
	offset = QPointF(50, 50);
	str = "Hello World";
}

void GraphicsText::setItemOffset(QPointF ponit)
{
	prepareGeometryChange(); // 通知场景图形改变
	offset = ponit;
	update();
}

QPointF GraphicsText::itemoffset()
{
	return offset;
}

void GraphicsText::setStrFont(int font)
{
	m_font.setPointSize(font);
}

void GraphicsText::setStr(const QString &s)
{
	str = s;
	update();
}

QSize GraphicsText::strSize()
{
	return getFontSize();
}

QRectF GraphicsText::boundingRect() const
{
	return QRectF(offset, getFontSize());
}

void GraphicsText::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	painter->setFont(m_font);
	if (m_font.pointSize() > 20)
	{
		painter->setPen(QPen(Qt::red));
	}
	else
	{
		painter->setPen(QPen(Qt::blue));
	}
	painter->drawText(offset, str);
}

QSize GraphicsText::getFontSize() const
{
	QFontMetrics fm(m_font);				   // 创建 QFontMetrics 对象
	int textWidth = fm.horizontalAdvance(str); // 计算字符串宽度
	int textHeight = fm.height();			   // 获取字体的高度
	return QSize(textWidth, textHeight);
}

#ifndef GRAPHICSTEXT_H
#define GRAPHICSTEXT_H

#include <QObject>
#include <QGraphicsObject>
#include <QFont>

class GraphicsText : public QGraphicsObject
{
	Q_OBJECT

public:
	GraphicsText();

public:
	QRectF boundingRect() const override;
	void setItemOffset(QPointF ponit);
	QPointF itemoffset();
	QSize strSize();
	void setStr(const QString &s);
	void setStrFont(int font);

private:
	void paint(QPainter *, const QStyleOptionGraphicsItem *, QWidget *) override;
	QSize getFontSize() const;

private:
	QString str;
	QPointF offset;
	QFont m_font;
};

#endif // GRAPHICSPIXMAP_H

#ifndef IMAGEVIEWWINDOW_H
#define IMAGEVIEWWINDOW_H

#include <QWidget>
#include <QMap>

class QTimer;
class QPropertyAnimation;
class GraphicsText;
class QGraphicsScene;
class QParallelAnimationGroup;
class GraphicsView;
class GraphicsScene;

class ImageViewWindow : public QWidget
{
	Q_OBJECT

public:
	ImageViewWindow(QWidget *parent = 0);
	~ImageViewWindow();
	QList<GraphicsText *> getItems() { return m_items; }
	void textChanged(QStringList &lsit);
	void setImage(const QString &str);

private:
	void initControl();

private:
	GraphicsScene *m_scene;
	QList<QMap<QString, QString>> m_textMapInfolst;
	QList<GraphicsText *> m_items;
	GraphicsView *m_view;
};

#endif // IMAGEVIEWWINDOW_H

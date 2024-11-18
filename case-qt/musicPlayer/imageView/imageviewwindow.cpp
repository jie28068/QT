#include "imageviewwindow.h"
#include "graphicsview.h"
#include "graphicstext.h"
#include "graphicsScene.h"

#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QDebug>
#include <QTimer>
#include <QVBoxLayout>

const int image_conunt = 5;
const int image_yoffset = 0;
const int image_xoffset = -200;
ImageViewWindow::ImageViewWindow(QWidget *parent)
	: QWidget(parent)
{
	initControl();
}

ImageViewWindow::~ImageViewWindow()
{
}

void ImageViewWindow::textChanged(QStringList &lsit)
{
	m_scene->clear();
	if (lsit.size() == 0)
	{
		auto textInfoMap = m_textMapInfolst[3];
		GraphicsText *item = new GraphicsText();
		item->setStr("歌词读取失败,请将歌词和音频文件放在同一文件下");
		item->setStrFont(textInfoMap["font"].toInt());
		item->setItemOffset(QPointF(0, textInfoMap["y"].toInt() + image_yoffset));
		item->setZValue(textInfoMap["index"].toInt());
		item->setOpacity(textInfoMap["opacity"].toFloat());
		m_items << item;
		m_scene->addItem(item);
		return;
	}
	if (lsit.size() <= 6)
	{
		lsit.append("");
		lsit.append("");
		lsit.append("");
		lsit.append("");
	}
	for (int index = 0; index < m_textMapInfolst.size(); index++)
	{
		const auto textInfoMap = m_textMapInfolst[index];
		GraphicsText *item = new GraphicsText();
		item->setStr(lsit[index]);
		item->setStrFont(textInfoMap["font"].toInt());
		item->setItemOffset(QPointF(textInfoMap["x"].toInt() + image_xoffset, textInfoMap["y"].toInt() + image_yoffset));
		item->setZValue(textInfoMap["index"].toInt());
		item->setOpacity(textInfoMap["opacity"].toFloat());
		m_items << item;
		m_scene->addItem(item);
	}
}

void ImageViewWindow::setImage(const QString &str)
{
	if (!str.isEmpty())
	{
		QPixmap backgroundImage(str);
		m_scene->setBackgroundImage(backgroundImage);
	}
}

void ImageViewWindow::initControl()
{
	// 场景
	m_scene = new GraphicsScene(QRect(0, 0, 876, 368));

	m_textMapInfolst << QMap<QString, QString>{
		{"index", "1"},
		{"font", "12"},
		{"y", "-100"},
		{"x", "300"},
		{"opacity", "0.2"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "2"},
		{"font", "15"},
		{"y", "0"},
		{"x", "300"},
		{"opacity", "0.4"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "3"},
		{"font", "18"},
		{"y", "100"},
		{"x", "300"},
		{"opacity", "0.6"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "4"},
		{"font", "25"},
		{"y", "200"},
		{"x", "300"},
		{"opacity", "1"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "3"},
		{"font", "18"},
		{"y", "300"},
		{"x", "300"},
		{"opacity", "0.6"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "2"},
		{"font", "15"},
		{"y", "400"},
		{"x", "300"},
		{"opacity", "0.4"}};
	m_textMapInfolst << QMap<QString, QString>{
		{"index", "1"},
		{"font", "12"},
		{"y", "500"},
		{"x", "300"},
		{"opacity", "0.2"}};

	QVBoxLayout *viewlayout = new QVBoxLayout(this);

	m_view = new GraphicsView(m_scene);
	m_view->setFrameShape(QFrame::NoFrame);
	m_view->setParent(this);
	m_view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate); // 刷新
	m_view->setCacheMode(QGraphicsView::CacheBackground);
	m_view->setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
	viewlayout->addWidget(m_view);
	setLayout(viewlayout);

	QPixmap backgroundImage(":/images/4");
	// backgroundItem->setPixmap(backgroundImage.scaled(m_scene->width(), m_scene->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
	m_scene->setBackgroundImage(backgroundImage);
}

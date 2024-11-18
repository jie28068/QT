#include "scene.h"
#include "rice.h"
#include "define.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsProxyWidget>
#include <QLabel>
#include <QVBoxLayout>

QList<QGraphicsItem *> DfinePro::itemsRice = QList<QGraphicsItem *>();

Scene::Scene(QObject *parent) : QGraphicsScene(parent)
{
    setSceneRect(-300, -300, 600, 600);
    setItemIndexMethod(QGraphicsScene::NoIndex); // 设置场景的项索引方法，NoIndex表示场景不使用索引来优化项的遍历和碰撞检测。
}

Scene::~Scene()
{
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // 获取鼠标点击的位置
    QPointF pos = event->scenePos();

    // 查找点击位置是否有item
    QGraphicsItem *item = itemAt(pos, QTransform());

    // 如果找到item，则删除该item
    if (item && event->button() == Qt::LeftButton && item->data(DfinePro::itemTypes) == DfinePro::MOUSE)
    {
        removeItem(item);
        delete item;
        // 发射 selectionChanged 信号，告知视图选定项已改变
        emit selectionChanged();
    }
}

void Scene::refreshMouse(int MouseCount)
{
    clear();
    DfinePro::itemsRice.clear();
    int count = MouseCount;
    if (MouseCount == 0)
        MouseCount = 7;
    for (int i = 0; i < MouseCount; ++i)
    {
        Mouse *mouse = new Mouse;
        mouse->setData(DfinePro::itemTypes, DfinePro::MOUSE);
        mouse->setPos(::sin((i * 6.28) / MouseCount) * 200,
                      ::cos((i * 6.28) / MouseCount) * 200);
        addItem(mouse);
    }
    if (count == 0)
        count = 5;
    for (int i = 0; i < count; ++i)
    {
        Rice *rice = new Rice;
        rice->setData(DfinePro::itemTypes, DfinePro::RICE);
        rice->setPos(::sin((i * 4.28) / count) * 100,
                     ::cos((i * 9.28) / count) * 100);
        addItem(rice);
        DfinePro::itemsRice.append(rice);
    }
}

void Scene::refreshRice(int count)
{
}

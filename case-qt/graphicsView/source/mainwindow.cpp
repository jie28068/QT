#include "../heard/mainwindow.h"
#include <QGraphicsRectItem>
#include <QInputDialog>
#include <QColorDialog>
#include <QFontDialog>
#include <QTime>
#include <QKeyEvent>
#include <QRandomGenerator>
#include <QAction>
#include <QToolBar>
#include <QGraphicsView>
#include <QWidget>
#include <QGridLayout>
#include <QColor>
#include <QWheelEvent>
#include <qdebug.h>
#include <QMenu>
#include <QPushButton>
#include <QDialog>
#include <QFileDialog>

template <class T>
void setBrushColor(T *item)
{
    QColor color = item->brush().color();
    color = QColorDialog::getColor(color, NULL, "选择填充颜色");
    if (color.isValid())
        item->setBrush(QBrush(color));
}

MyMainWindow::MyMainWindow(QWidget *parent) : QMainWindow(parent), status(1)
{
    setWindowIcon(QIcon(":images/824"));
    InitAction();
    InitActionList();
    InitUI();
    InitConnect();
}

MyMainWindow::~MyMainWindow()
{
}

void MyMainWindow::InitActionList()
{
    actionList = new QActionGroup(this);
    actionList->addAction(action_magnify);
    actionList->addAction(action_lessen);
    actionList->addAction(action_recover);
    actionList->addAction(action_leftSpin);
    actionList->addAction(action_rightSpin);
    actionList->addAction(action_preposition);
    actionList->addAction(action_postposition);
    actionList->addAction(action_group);
    actionList->addAction(action_beater);
    actionList->addAction(action_delete);
    actionList->addAction(action_exit);
    actionList->addAction(action_rectangle);
    actionList->addAction(action_oval);
    actionList->addAction(action_round);
    actionList->addAction(action_triangle);
    actionList->addAction(action_trapezoid);
    actionList->addAction(action_line);
    actionList->addAction(action_text);
    actionList->addAction(action_backgroundColor);
    actionList->addAction(action_status);
    actionList->addAction(action_output);
    actionList->addAction(action_input);
}

void MyMainWindow::InitAction()
{
    signalMapper = new QSignalMapper(this);

    action_magnify = new QAction("放大", this);
    action_magnify->setToolTip("放大");
    action_magnify->setIcon(QIcon(":/images/zoomin"));

    action_lessen = new QAction("缩小", this);
    action_lessen->setToolTip("缩小");
    action_lessen->setIcon(QIcon(":/images/zoomout"));

    action_recover = new QAction("恢复", this);
    action_recover->setToolTip("恢复");
    action_recover->setIcon(QIcon(":/images/420"));

    action_leftSpin = new QAction("左旋转", this);
    action_leftSpin->setToolTip("左旋转");
    action_leftSpin->setIcon(QIcon(":/images/rotateleft"));

    action_rightSpin = new QAction("右旋转", this);
    action_rightSpin->setToolTip("右旋转");
    action_rightSpin->setIcon(QIcon(":/images/rotateright"));

    action_preposition = new QAction("前置", this);
    action_preposition->setToolTip("前置");
    action_preposition->setIcon(QIcon(":/images/526"));

    action_postposition = new QAction("后置", this);
    action_postposition->setToolTip("后置");
    action_postposition->setIcon(QIcon(":/images/528"));

    action_group = new QAction("组合", this);
    action_group->setToolTip("组合");
    action_group->setIcon(QIcon(":/images/UNGROUP"));

    action_beater = new QAction("打散", this);
    action_beater->setToolTip("打散");
    action_beater->setIcon(QIcon(":/images/128"));

    action_delete = new QAction("删除", this);
    action_delete->setToolTip("删除");
    action_delete->setIcon(QIcon(":/images/108"));

    action_exit = new QAction("退出", this);
    action_exit->setToolTip("退出");
    action_exit->setIcon(QIcon(":/images/132"));

    action_rectangle = new QAction("矩形", this);
    action_rectangle->setToolTip("矩形");
    action_rectangle->setIcon(QIcon(":/images/RECTANGL"));

    action_oval = new QAction("椭圆", this);
    action_oval->setToolTip("椭圆");
    action_oval->setIcon(QIcon(":/images/ELLIPSE"));

    action_round = new QAction("圆形", this);
    action_round->setToolTip("圆形");
    action_round->setIcon(QIcon(":/images/08"));

    action_triangle = new QAction("三角形", this);
    action_triangle->setToolTip("三角形");
    action_triangle->setIcon(QIcon(":/images/Icon1242"));

    action_trapezoid = new QAction("多边形", this);
    action_trapezoid->setToolTip("多边形");
    action_trapezoid->setIcon(QIcon(":/images/FREEFORM"));

    action_line = new QAction("直线", this);
    action_line->setToolTip("直线");
    action_line->setIcon(QIcon(":/images/LINE"));

    action_text = new QAction("文字", this);
    action_text->setToolTip("文字");
    action_text->setIcon(QIcon(":/images/800"));

    action_backgroundColor = new QAction("设置背景颜色", this);
    action_backgroundColor->setIcon(QIcon(":/images/212"));

    action_status = new QAction("切换绘制模式", this);
    action_status->setIcon(QIcon(":/images/hmsetup"));

    action_output = new QAction("导出", this);
    action_output->setIcon(QIcon(":/images/fileprint"));

    action_input = new QAction("导入", this);
    action_input->setIcon(QIcon(":/images/224"));
}

void MyMainWindow::InitUI()
{
    myCentelWidget = new QWidget(this);
    // 创建状态栏标签
    statusBar = new QStatusBar(this);
    setStatusBar(statusBar);
    labViewCord = new QLabel("View 坐标:");
    labViewCord->setMinimumWidth(150);
    statusBar->addWidget(labViewCord);

    labSceneCord = new QLabel("Scene 坐标:");
    labSceneCord->setMinimumWidth(150);
    statusBar->addWidget(labSceneCord);

    labItemCord = new QLabel("Item 坐标:");
    labItemCord->setMinimumWidth(150);
    statusBar->addWidget(labItemCord);

    labItemInfo = new QLabel("ItemInfo:");
    labItemInfo->setMinimumWidth(100);
    statusBar->addWidget(labItemInfo);

    labSceneScale = new QLabel("场景缩放:");
    labSceneScale->setMinimumWidth(100);
    statusBar->addWidget(labSceneScale);
    // end

    hToolBar = new QToolBar(this);
    hToolBar->addAction(action_magnify);
    hToolBar->addAction(action_lessen);
    hToolBar->addAction(action_recover);
    hToolBar->addSeparator();
    hToolBar->addAction(action_leftSpin);
    hToolBar->addAction(action_rightSpin);
    hToolBar->addAction(action_preposition);
    hToolBar->addAction(action_postposition);
    hToolBar->addAction(action_group);
    hToolBar->addAction(action_beater);
    hToolBar->addSeparator();
    hToolBar->addAction(action_delete);
    hToolBar->addSeparator();
    hToolBar->addAction(action_output);
    hToolBar->addAction(action_input);
    hToolBar->addSeparator();
    hToolBar->addAction(action_exit);

    vToolBar = new QToolBar(this);
    vToolBar->addAction(action_rectangle);
    vToolBar->addAction(action_oval);
    vToolBar->addAction(action_round);
    vToolBar->addAction(action_triangle);
    vToolBar->addAction(action_trapezoid);
    vToolBar->addAction(action_line);
    vToolBar->addAction(action_text);
    vToolBar->addSeparator();
    vToolBar->addAction(action_status);

    hToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    vToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    vToolBar->setAllowedAreas(Qt::LeftToolBarArea);
    addToolBar(Qt::TopToolBarArea, hToolBar);
    addToolBar(Qt::LeftToolBarArea, vToolBar);

    // view设置
    view = new MyGraphicsView(myCentelWidget);
    view->setGeometry(QRect(0, 0, 600, 380));
    view->setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing); // 锯齿设置
    view->setDragMode(QGraphicsView::RubberBandDrag);                          // 设置拖动模式为橡皮筋拖动
    view->setCursor(Qt::ArrowCursor);                                          // 设置鼠标形状为十字光标
    view->setMouseTracking(true);                                              // 启用鼠标跟踪
    setCentralWidget(view);
    scene = new QGraphicsScene(0, 0, 600, 380); // 创建QGraphicsScene
    view->setScene(scene);                      // 关联view
    // end
}

void MyMainWindow::InitConnect()
{
    // view
    connect(view, &MyGraphicsView::mouseMovePoint, this, [=](QMouseEvent *event)
            {
                QPoint point = event->pos();
                labViewCord->setText(QString("View 坐标：%1,%2").arg(point.x()).arg(point.y()));
                QPointF pointScene = view->mapToScene(point); 
                currentMousePos = pointScene;
                labSceneCord->setText(QString::asprintf("Scene 坐标：%.0f,%.0f", pointScene.x(), pointScene.y())); });
    connect(view, &MyGraphicsView::mouseClicked, this, &MyMainWindow::on_mouseClicked);
    connect(view, &MyGraphicsView::mouseDoubleClick, this, &MyMainWindow::on_mouseDoubleClick);
    connect(view, &MyGraphicsView::keyPress, this, &MyMainWindow::on_keyPress);
    connect(view, &MyGraphicsView::onwheelEvent, this, &MyMainWindow::on_wheelEvent);
    connect(view, &MyGraphicsView::mouseClickedRight, this, &MyMainWindow::on_mouseClickedRight);
    // end

    // action
    connect(signalMapper, &QSignalMapper::mappedObject, this, &MyMainWindow::on_ActionTriggered);
    for (auto action : actionList->actions())
    {
        signalMapper->setMapping(action, action);
        connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    }
}

void MyMainWindow::on_ActionTriggered(QObject *obj)
{
    QAction *action = qobject_cast<QAction *>(obj);
    if (action == action_magnify)
    {
        on_action_magnify();
    }
    else if (action == action_lessen)
    {
        on_action_lessen();
    }
    else if (action == action_recover)
    {
        on_action_recover();
    }
    else if (action == action_leftSpin)
    {
        on_action_leftSpin();
    }
    else if (action == action_rightSpin)
    {
        on_action_rightSpin();
    }
    else if (action == action_preposition)
    {
        on_action_preposition();
    }
    else if (action == action_postposition)
    {
        on_action_postposition();
    }
    else if (action == action_group)
    {
        on_action_group();
    }
    else if (action == action_beater)
    {
        on_action_beater();
    }
    else if (action == action_delete)
    {
        on_action_delete();
    }
    else if (action == action_exit)
    {
        on_action_exit();
    }
    else if (action == action_rectangle)
    {
        on_action_rectangle();
    }
    else if (action == action_oval)
    {
        on_action_oval();
    }
    else if (action == action_round)
    {
        on_action_round();
    }
    else if (action == action_triangle)
    {
        on_action_triangle();
    }
    else if (action == action_trapezoid)
    {
        on_action_trapezoid();
    }
    else if (action == action_line)
    {
        on_action_line();
    }
    else if (action == action_text)
    {
        on_action_text();
    }
    else if (action == action_backgroundColor)
    {
        on_action_backgroundColor();
    }
    else if (action == action_status)
    {
        on_action_status();
    }
    else if (action == action_output)
    {
        on_action_output();
    }
    else if (action == action_input)
    {
        on_action_input();
    }
}

void MyMainWindow::on_mouseDoubleClick(QMouseEvent *event)
{
    QPointF pointSecene = view->mapToScene(event->pos());
    QGraphicsItem *item = nullptr;
    item = scene->itemAt(pointSecene, view->transform());
    if (item)
    {
        switch (item->type())
        {
        case QGraphicsRectItem::Type: // 矩形框
        {
            QGraphicsRectItem *theItem = qgraphicsitem_cast<QGraphicsRectItem *>(item);
            setBrushColor(theItem);
            break;
        }
        case QGraphicsEllipseItem::Type: // 椭圆和圆
        {
            QGraphicsEllipseItem *theItem = qgraphicsitem_cast<QGraphicsEllipseItem *>(item);
            setBrushColor(theItem);
            break;
        }
        case QGraphicsPolygonItem::Type: // 梯形和三角形
        {
            QGraphicsPolygonItem *theItem = qgraphicsitem_cast<QGraphicsPolygonItem *>(item);
            setBrushColor(theItem);
            break;
        }
        case QGraphicsLineItem::Type: // 直线，设置线条颜色
        {
            QGraphicsLineItem *theItem = qgraphicsitem_cast<QGraphicsLineItem *>(item);
            QPen pen = theItem->pen();
            QColor color = theItem->pen().color();
            color = QColorDialog::getColor(color, this, "选择线条颜色");
            if (color.isValid())
            {
                pen.setColor(color);
                theItem->setPen(pen);
            }
            break;
        }
        case QGraphicsTextItem::Type: // 文字，设置字体
        {
            QGraphicsTextItem *theItem = qgraphicsitem_cast<QGraphicsTextItem *>(item);

            QDialog *log = new QDialog(this);
            log->setWindowTitle("文本设置");
            QPushButton *label = new QPushButton("修改文本", log);
            QString text = theItem->toPlainText();
            QLineEdit *lineEdit = new QLineEdit(text, log);
            QPushButton *fontButton = new QPushButton("设置字体", log);
            QPushButton *colorButton = new QPushButton("设置字体颜色", log);
            QGridLayout *layout = new QGridLayout(log);
            layout->addWidget(label, 0, 1);
            layout->addWidget(lineEdit, 0, 0);
            layout->addWidget(fontButton, 1, 0);
            layout->addWidget(colorButton, 1, 1);
            log->setLayout(layout);
            log->open();

            connect(label, &QPushButton::clicked, [=]()
                    { QString text = theItem->toPlainText();
                    theItem->setPlainText(lineEdit->text()); });
            connect(fontButton, &QPushButton::clicked, [=]()
                    {            QFont font = theItem->font();
            bool ok = false;
            font = QFontDialog::getFont(&ok, font, this, "设置字体");
            if (ok)
                theItem->setFont(font); });
            connect(colorButton, &QPushButton::clicked, [=]()
                    {
                QColor color = QColorDialog::getColor(color, NULL, "选择填充颜色");
                if (color.isValid())
                    theItem->setDefaultTextColor(color); });

            break;
        }
        default:
            break;
        }
    }
    else
    {
        on_action_text();
    }
}

void MyMainWindow::on_mouseClicked(QMouseEvent *event)
{
    QPointF pointScene = view->mapToScene(event->pos());
    QGraphicsItem *item = nullptr;
    // 获取光标下的绘图项
    item = scene->itemAt(pointScene, view->transform());
    if (item)
    {
        QPointF pointItem = item->mapFromScene(pointScene); // 转换为绘图项的局部坐标
        labItemCord->setText(QString::asprintf("Item 坐标：%.0f,%.0f", pointItem.x(), pointItem.y()));
        labItemInfo->setText(item->data(ItemDesciption).toString() + ", ItemId=" +
                             item->data(ItemId).toString());
    }
}

void MyMainWindow::on_mouseClickedRight(QMouseEvent *event)
{
    QPointF pointScene = view->mapToScene(event->pos());
    QGraphicsItem *item = nullptr;
    // 获取光标下的绘图项
    item = scene->itemAt(pointScene, view->transform());
    if (item)
    {
        QMenu *menu = new QMenu(this);
        menu->addAction(action_magnify);
        menu->addAction(action_lessen);
        menu->exec(QCursor::pos());
    }
    else
    {
        QMenu *menu = new QMenu(this);
        menu->addAction(action_backgroundColor);
        menu->exec(QCursor::pos());
    }
}

void MyMainWindow::on_keyPress(QKeyEvent *event)
{
    if (scene->selectedItems().count() != 1)
        return;
    QGraphicsItem *item = scene->selectedItems().at(0);
    if (event->key() == Qt::Key_Up)
    {
        item->setY(-1 + item->y());
    }
    else if (event->key() == Qt::Key_Down)
    {
        item->setY(1 + item->y());
    }
    else if (event->key() == Qt::Key_Left)
    {
        item->setX(-1 + item->x());
    }
    else if (event->key() == Qt::Key_Right)
    {
        item->setX(1 + item->x());
    }
    else if (event->key() == Qt::Key_Right)
    {
        item->setX(1 + item->x());
    }
    else if (event->key() == Qt::Key_PageDown)
    {
        item->setScale(-0.1 + item->scale());
    }
    else if (event->key() == Qt::Key_PageUp)
    {
        item->setScale(0.1 + item->scale());
    }
    else if (event->key() == Qt::Key_Space)
    {
        item->setRotation(90 + item->rotation());
    }
    else if (event->key() == Qt::Key_Delete)
    {
        scene->removeItem(item);
    }
}

void MyMainWindow::on_wheelEvent(QWheelEvent *event)
{
    if (event->modifiers() == Qt::ControlModifier)
    {
        int count = scene->selectedItems().count();
        if (count == 1)
        {
            QGraphicsItem *item = scene->selectedItems().at(0);
            if (event->angleDelta().y() > 0)
            {
                if (item->data(ItemScale).toInt() < 80)
                {
                    item->setData(ItemScale, item->data(ItemScale).toInt() + 10);
                    item->setScale(0.1 + item->scale());
                }
            }
            else
            {
                if (item->data(ItemScale).toInt() > -80)
                {
                    item->setData(ItemScale, item->data(ItemScale).toInt() - 10);
                    item->setScale(item->scale() - 0.1);
                }
            }
            labSceneScale->setText(QString("图元缩放：%1%").arg(item->data(ItemScale).toInt()));
        }
        else
        {
            static int scale = 0;
            if (event->angleDelta().y() > 0)
            {

                if (scale < 80)
                {
                    scale += 10;
                    view->scale(1.1, 1.1);
                }
            }
            else
            {
                if (scale > -80)
                {
                    scale -= 10;
                    view->scale(0.9, 0.9);
                }
            }
            labSceneScale->setText(QString("场景缩放：%1%").arg(scale));
        }
    }
    // 获取当前鼠标位置
    QPointF mousePos = event->position();
    // 将鼠标位置转换为场景坐标系下的位置
    QPointF scenePos = view->mapToScene(mousePos.toPoint());
    // 以鼠标位置为中心
    view->centerOn(scenePos);
}

void MyMainWindow::on_action_magnify()
{
    int count = scene->selectedItems().count();
    if (count == 1)
    {
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setScale(0.1 + item->scale());
    }
    else
    {
        view->scale(1.1, 1.1);
    }
}

void MyMainWindow::on_action_lessen()
{
    int count = scene->selectedItems().count();
    if (count == 1)
    {
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setScale(-0.1 + item->scale());
    }
    else
    {
        view->scale(0.9, 0.9);
    }
}

void MyMainWindow::on_action_recover()
{
    int count = scene->selectedItems().count();
    if (count == 1)
    {
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setRotation(0);
        item->setScale(1.0);
    }
    else
    {
        view->resetTransform();
    }
}

void MyMainWindow::on_action_leftSpin()
{
    int cnt = scene->selectedItems().count();
    if (cnt == 1)
    {
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setRotation(-30 + item->rotation());
    }
    else
        view->rotate(-30);
}

void MyMainWindow::on_action_rightSpin()
{
    int cnt = scene->selectedItems().count();
    if (cnt == 1)
    {
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setRotation(30 + item->rotation());
    }
    else
        view->rotate(30);
}

void MyMainWindow::on_action_preposition()
{
    int cnt = scene->selectedItems().count();
    if (cnt > 0)
    { // 只处理选中的第1个绘图项
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setZValue(++frontZ);
    }
}

void MyMainWindow::on_action_postposition()
{
    int cnt = scene->selectedItems().count();
    if (cnt > 0)
    { // 只处理选中的第1个绘图项
        QGraphicsItem *item = scene->selectedItems().at(0);
        item->setZValue(--backZ);
    }
}

void MyMainWindow::on_action_group()
{
    int cnt = scene->selectedItems().count();
    if (cnt > 1)
    {
        QGraphicsItemGroup *group = new QGraphicsItemGroup; // 创建组合
        scene->addItem(group);                              // 组合添加到场景中

        for (int i = 0; i < cnt; i++)
        {
            QGraphicsItem *item = scene->selectedItems().at(0);
            item->setSelected(false); // 清除选择虚线框
            item->clearFocus();
            group->addToGroup(item); // 添加到组合,item会不在scene上
        }
        group->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);

        group->setZValue(++frontZ);
        scene->clearSelection();
        group->setSelected(true);
    }
}

void MyMainWindow::on_action_beater()
{
    int cnt = scene->selectedItems().count();
    if (cnt == 1)
    {
        QGraphicsItemGroup *group;
        group = (QGraphicsItemGroup *)scene->selectedItems().at(0);
        scene->destroyItemGroup(group);
    }
}

void MyMainWindow::on_action_delete()
{
    int cnt = scene->selectedItems().count();
    if (cnt > 0)
        for (int i = 0; i < cnt; i++)
        {
            QGraphicsItem *item = scene->selectedItems().at(0);
            scene->removeItem(item); // 删除绘图项
        }
}

void MyMainWindow::on_action_exit()
{
    exit(0);
}

void MyMainWindow::on_action_rectangle()
{
    if (status == Gloabine::polygon)
    {
        QGraphicsRectItem *item = new QGraphicsRectItem(50, 50, 100, 50); // x,y 为左上角的图元局部坐标，图元中心点为0,0
        item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
        item->setBrush(QBrush(Qt::yellow));
        item->setZValue(++frontZ);

        item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                     QRandomGenerator::global()->bounded(-50, 50)); // 生成一个随机的位置，x和y坐标的范围都是-50到50之间

        item->setData(ItemId, ++seqNum);
        item->setData(ItemDesciption, "矩形");
        item->setData(ItemScale, 0);

        scene->addItem(item);
        scene->clearSelection();
        item->setSelected(true);
        view->setCursor(Qt::ArrowCursor);
    }
    else
    {
    }
}

void MyMainWindow::on_action_oval()
{
    QGraphicsEllipseItem *item = new QGraphicsEllipseItem(50, 30, 100, 60);
    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    item->setBrush(QBrush(Qt::blue)); // 填充颜色
    item->setZValue(++frontZ);        // 用于叠放顺序
    //    item->setPos(-50+(qrand() % 100),-50+(qrand() % 100)); //初始位置, qrand()函数过时
    item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                 QRandomGenerator::global()->bounded(-50, 50));

    item->setData(ItemId, ++seqNum);       // 自定义数据，ItemId键
    item->setData(ItemDesciption, "椭圆"); // 自定义数据，ItemDesciption键
    item->setData(ItemScale, 0);

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_round()
{
    QGraphicsEllipseItem *item = new QGraphicsEllipseItem(50, 50, 100, 100);
    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    item->setBrush(QBrush(Qt::cyan));
    item->setZValue(++frontZ);
    //    item->setPos(-50+(qrand() % 100),-50+(qrand() % 100));
    item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                 QRandomGenerator::global()->bounded(-50, 50));

    item->setData(ItemId, ++seqNum);
    item->setData(ItemDesciption, "圆形");
    item->setData(ItemScale, 0);

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_triangle()
{
    QGraphicsPolygonItem *item = new QGraphicsPolygonItem;
    QPolygonF points;
    points.append(QPointF(80, 80));
    points.append(QPointF(40, 160));
    points.append(QPointF(120, 160));
    item->setPolygon(points);
    //    item->setPos(-50+(qrand() % 100),-50+(qrand() % 100));
    item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                 QRandomGenerator::global()->bounded(-50, 50));

    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    item->setBrush(QBrush(Qt::magenta));
    item->setZValue(++frontZ);

    item->setData(ItemId, ++seqNum);
    item->setData(ItemDesciption, "三角形");
    item->setData(ItemScale, 0);

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_trapezoid()
{
    QGraphicsPolygonItem *item = new QGraphicsPolygonItem;

    QPolygonF points;
    points.append(QPointF(-40, -40));
    points.append(QPointF(40, -40));
    points.append(QPointF(100, 40));
    points.append(QPointF(-100, 40));
    item->setPolygon(points);
    //    item->setPos(-50+(qrand() % 100),-50+(qrand() % 100));
    item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                 QRandomGenerator::global()->bounded(-50, 50));

    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    item->setBrush(QBrush(Qt::green));
    item->setZValue(++frontZ);

    item->setData(ItemId, ++seqNum);
    item->setData(ItemDesciption, "梯形");
    item->setData(ItemScale, 0);

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_line()
{
    QGraphicsLineItem *item = new QGraphicsLineItem(100, 100, 100, 0); // x,y 为左上角的图元局部坐标，图元中心点为0,0
    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);

    QPen pen(Qt::red);
    pen.setWidth(3);
    item->setPen(pen);

    item->setZValue(++frontZ);
    item->setPos(QRandomGenerator::global()->bounded(-50, 50),
                 QRandomGenerator::global()->bounded(-50, 50));

    item->setData(ItemId, ++seqNum);
    item->setData(ItemDesciption, "直线");
    item->setData(ItemScale, 0);

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_text()
{
    QString str = QInputDialog::getText(this, "输入文字", "请输入文字");
    if (str.isEmpty())
        return;
    QGraphicsTextItem *item = new QGraphicsTextItem(str);
    QFont font = this->font();
    font.setPointSize(20);
    font.setBold(true);
    item->setFont(font);

    item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable);
    item->setPos(currentMousePos);
    item->setZValue(++frontZ);

    item->setData(ItemId, ++seqNum);
    item->setData(ItemDesciption, "文字");

    scene->addItem(item);
    scene->clearSelection();
    item->setSelected(true);
}

void MyMainWindow::on_action_backgroundColor()
{
    QColor color = QColorDialog::getColor(Qt::white, NULL, "选择填充颜色");
    if (color.isValid())
        scene->setBackgroundBrush(QBrush(color));
}

void MyMainWindow::on_action_status()
{
    if (status == Gloabine::custom)
    {
        status = Gloabine::polygon;
        view->setStatus(Gloabine::polygon);
    }
    else
    {
        status = Gloabine::custom;
        view->setStatus(Gloabine::custom);
    }
}

void MyMainWindow::on_action_output()
{
    QPixmap pix(view->sceneRect().width() - 10,
                view->sceneRect().height() - 10);
    pix.fill(Qt::white);
    QPainter painter(&pix);
    painter.setRenderHint(QPainter::Antialiasing);
    scene->render(&painter);
    painter.end();
    QString filePath =
        QFileDialog::getSaveFileName(&QWidget(), tr("导出图片"), "", "BMP(*.bmp);;PNG(*.png)");
    if (filePath == "" && pix.isNull())
        return;
    pix.save(filePath);
}

void MyMainWindow::on_action_input()
{
    QString imagePath = QFileDialog::getOpenFileName(&QWidget(), tr("导出图片"), "", "PNG(*.png);;BMP(*.bmp)");
    QPixmap imagePixmap(imagePath);
    MyGraphicsPixmapItem *pixmapItem = new MyGraphicsPixmapItem(imagePixmap);
    pixmapItem->setFlag(QGraphicsItem::ItemIsMovable);
    pixmapItem->setFlag(QGraphicsItem::ItemIsSelectable);
    scene->addItem(pixmapItem);
}
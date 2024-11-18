#include "mianWindows.h"

#include <QGraphicsView>
#include <QTimer>
#include <QtWidgets>

static const int MouseCount = 7;
MianWindows::MianWindows(Scene *scene, QWidget *parent) : QMainWindow(parent), m_scene(scene), number(0)
{
    setMinimumSize(500, 300);
    initUI();
}

MianWindows::~MianWindows()
{
}

void MianWindows::initUI()
{
    QGraphicsView *view = new QGraphicsView(m_scene);
    view->setRenderHint(QPainter::Antialiasing); // 设置视图的渲染提示，开启抗锯齿，以提高绘制质量
    view->setBackgroundBrush(QPixmap(":/images/cheese.jpg"));
    view->setCacheMode(QGraphicsView::CacheBackground);                     // 设置视图的缓存模式，缓存背景以提高性能。
    view->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate); // 设置视图的视口更新模式，只有当项的边界矩形发生变化时才更新视口
    // view->setDragMode(QGraphicsView::ScrollHandDrag);                       // 设置视图的拖动模式，允许用户通过拖动来移动视图
    view->setCursor(Qt::CrossCursor);
    view->setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    view->resize(400, 300);
    // 创建一个标签
    QLabel *label = new QLabel("抓获数:");
    // 创建一个按钮
    QPushButton *button = new QPushButton("重新开始");
    label2 = new QLabel;
    // 创建一个 QWidget，用于容纳 Label 和 Button
    QWidget *rightWidget = new QWidget();
    // 创建布局并添加控件
    QGridLayout *layout = new QGridLayout();
    layout->addWidget(label, 0, 0);
    layout->addWidget(label2, 0, 1);
    layout->addWidget(button, 1, 0);
    rightWidget->setLayout(layout);
    rightWidget->setBaseSize(100, 300);
    // 设置主窗口的布局
    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addWidget(view);
    mainLayout->addWidget(rightWidget);
    // 设置主窗口的中心小部件
    QWidget *centralWidget = new QWidget();
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    connect(m_scene, &QGraphicsScene::selectionChanged, this, &MianWindows::onLabel);
    connect(button, &QPushButton::clicked, m_scene, &Scene::refreshMouse);
    // connect(button, &QPushButton::clicked, m_scene, &Scene::refreshRice);
    connect(button, &QPushButton::clicked, [&]()
            {label2->setText("0");number=0; });
}

void MianWindows::onLabel()
{
    label2->setText(QString("%1").arg(++number));
}
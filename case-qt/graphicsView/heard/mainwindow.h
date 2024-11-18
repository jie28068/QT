#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QGraphicsScene>
#include <QLabel>
#include <QStatusBar>
#include <QGraphicsView>
#include <QGraphicsScene>
#include "qwgraphicsview.h"
#include "MyGraphicsPixmapItem.h"
#include <QSignalMapper>
#include <QActionGroup>
#include "MyQGraphicsItem.h"
#include "Globine.h"
class MyQGraphicsItem;
class MyMainWindow : public QMainWindow
{
    Q_OBJECT
private:
    static const int ItemId = 1;         // 绘图项自定义数据的key
    static const int ItemDesciption = 2; // 绘图项自定义数据的key
    static const int ItemScale = 3;      // 绘图项自定义数据的key

    int seqNum = 0;
    int backZ = 0;
    int frontZ = 0;

    int status; // 绘制枚举类型

public:
    explicit MyMainWindow(QWidget *parent = 0);
    ~MyMainWindow();
    void InitAction();
    void InitUI();
    void InitConnect();
    void InitActionList();

private slots:

    void on_ActionTriggered(QObject *obj);
    void on_mouseDoubleClick(QMouseEvent *event);
    void on_mouseClickedRight(QMouseEvent *event);
    void on_mouseClicked(QMouseEvent *event);
    void on_keyPress(QKeyEvent *event);
    void on_wheelEvent(QWheelEvent *event);

    void on_action_magnify();
    void on_action_lessen();
    void on_action_recover();
    void on_action_leftSpin();
    void on_action_rightSpin();
    void on_action_preposition();
    void on_action_postposition();
    void on_action_group();
    void on_action_beater();
    void on_action_delete();
    void on_action_exit();
    void on_action_rectangle();
    void on_action_oval();
    void on_action_round();
    void on_action_triangle();
    void on_action_trapezoid();
    void on_action_line();
    void on_action_text();
    void on_action_backgroundColor();
    void on_action_status();
    void on_action_output();
    void on_action_input();

private:
    QStatusBar *statusBar;
    QLabel *labViewCord;   // View 坐标
    QLabel *labSceneCord;  // Scene 坐标
    QLabel *labItemCord;   // Item 坐标
    QLabel *labItemInfo;   // ItemInfo
    QLabel *labSceneScale; // 场景缩放

    QToolBar *hToolBar; // 水平工具栏
    QToolBar *vToolBar; // 垂直工具栏

    QWidget *myCentelWidget; // 中心窗口
    MyGraphicsView *view;
    QGraphicsScene *scene;
    QPointF currentMousePos; // 当前鼠标位置

    QSignalMapper *signalMapper;
    QActionGroup *actionList;
    QAction *action_magnify;         // 放大
    QAction *action_lessen;          // 缩小
    QAction *action_recover;         // 恢复
    QAction *action_leftSpin;        // 左旋转
    QAction *action_rightSpin;       // 右旋转
    QAction *action_preposition;     // 前置
    QAction *action_postposition;    // 后置
    QAction *action_group;           // 组合
    QAction *action_beater;          // 打散
    QAction *action_delete;          // 删除
    QAction *action_exit;            // 退出
    QAction *action_rectangle;       // 矩形
    QAction *action_oval;            // 椭圆
    QAction *action_round;           // 圆形
    QAction *action_triangle;        // 三角形
    QAction *action_trapezoid;       // 梯形
    QAction *action_line;            // 直线
    QAction *action_text;            // 文字
    QAction *action_backgroundColor; // 背景颜色
    QAction *action_status;          // 切换绘制状态
    QAction *action_output;          // 导出
    QAction *action_input;           // 导入
};
#endif
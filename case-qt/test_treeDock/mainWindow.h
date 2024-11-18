#pragma once

#include <QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QFileDialog>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    // 枚举类型treeItemType， 用于创建 QTreeWidgetItem 时作为节点的type, 自定义类型必须大于1000
    // itTopItem 顶层节点;  itGroupItem 组节点； itImageItem 图片
    enum treeItemType
    {
        itTopItem = 1001,
        itGroupItem,
        itImageItem
    };

    // 枚举类型，表示列号
    enum treeColNum
    {
        colItem = 0,
        colItemType = 1
    }; // 目录树列的编号定义

    MainWindow(QWidget *parent = nullptr);
    void initUI();
    ~MainWindow();
    void retranslateUi();
    void iniTree(); // 目录树初始化
    void addFolderItem(QTreeWidgetItem *parItem, QString dirName);
    QString getFinalFolderName(const QString &fullPathName);
    void addImageItem(QTreeWidgetItem *parItem, QString aFilename);
    void displayImage(QTreeWidgetItem *item);
    void changeItemCaption(QTreeWidgetItem *item);

private slots:
    void on_treeFiles_currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);
    void on_actAddFolder_triggered();
    void on_actAddFiles_triggered();
    void on_actZoomOut_triggered(); // 缩小，zoom out

    void on_actZoomIn_triggered(); // 放大，zoom in

    void on_actZoomFitW_triggered(); // 适合宽度

    void on_actZoomFitH_triggered(); // 适合高度

    void on_actZoomRealSize_triggered(); // 实际大小

    void on_actDeleteItem_triggered(); // 删除节点

    void on_actScanItems_triggered(); // 遍历节点

    void on_actDockVisible_toggled(bool arg1);

    void on_dockWidget_visibilityChanged(bool visible);

    void on_dockWidget_topLevelChanged(bool topLevel);

    void on_actDockFloat_triggered(bool checked);

private:
    QLabel *LabFileName;
    QPixmap curPixmap; // 当前的图片
    float pixRatio;    // 当前图片缩放比例
    QAction *actAddFolder;
    QAction *actAddFiles;
    QAction *actZoomIn;
    QAction *actZoomOut;
    QAction *actZoomRealSize;
    QAction *actZoomFitW;
    QAction *actDeleteItem;
    QAction *actQuit;
    QAction *actZoomFitH;
    QAction *actScanItems;
    QAction *actDockVisible;
    QAction *actDockFloat;
    QWidget *centralWidget;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QLabel *LabPicture;
    QMenuBar *menuBar;
    QMenu *menuPic;
    QMenu *menuView;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QTreeWidget *treeFiles;
};
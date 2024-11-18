#pragma once

#include "puzzlewidget.h"
#include "piecesmodel.h"
#include <QPixmap>
#include <QMainWindow>
#include <QListView>
#include <QMenuBar>
#include <QCoreApplication>
#include <QStandardPaths>
#include <QFileDialog>
#include <QStringList>
#include <QImageReader>
#include <QMessageBox>
#include <QHBoxLayout>

class PuzzleWidget;
class PiecesModel;

class MainWindow : public QMainWindow
{

public:
    MainWindow(QWidget *parent = nullptr);

public slots:
    // 打开图片
    void openImage();
    // 重置图片
    void setupPuzzle();
    // 加载图片
    void loadImage(const QString &path);

private slots:
    // 完成
    void setCompleted();

private:
    // 初始化菜单栏
    void setupMenus();
    // 初始化图片
    void setupWidgets();

    QPixmap puzzleImage;
    QListView *piecesList;
    PuzzleWidget *puzzleWidget;
    PiecesModel *model;
};
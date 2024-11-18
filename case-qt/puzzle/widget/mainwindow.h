/*
 * @Author: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @Date: 2024-01-05 15:17:45
 * @LastEditors: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @LastEditTime: 2024-01-16 15:01:13
 * @FilePath: \puzzle\widget\mainwindow.h
 */

#pragma once

#include "puzzlewidget.h"
#include "../model/piecesmodel.h"
#include "beginwidget.h"

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
#include <QTextEdit>

class PuzzleWidget;
class PiecesModel;
class ImageDialog;

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
    /// @brief 开始游戏
    void beginGame();

private slots:
    // 完成
    void setCompleted(bool iscompleted = true);
    /// @brief 查看初始图
    void viewOriginalImage();
    /// @brief 计时器
    void timeUpdate();

private:
    // 初始化菜单栏
    void setupbtn();
    // 初始化图片
    void setupWidgets();

    void originalIamgeDialog();

    QPixmap puzzleImage;
    QListView *piecesList;
    PuzzleWidget *puzzleWidget;
    PiecesModel *model;
    Beginwidget *beginwidget;

    QTimer *countDownTimer;  // 计时器
    int countDownTimerValue; // 计数
    QLabel *timerLabel;
    QLabel *labelImage; // 查看原图
    QDialog *dialog;
};
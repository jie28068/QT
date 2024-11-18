#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QMainWindow>
#include "jsonDataManager.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void initUI();
};
#endif
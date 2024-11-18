#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QToolBar>
#include <QPushButton>
#include <QLabel>
#include <QTableWidget>
#include <QDragEnterEvent>
#include <QMap>

class QtVariantProperty;
class QtProperty;
class QtBrowserIndex;
class CustomDockWidget;
class ControlDialogTreeView;
class MainWindow : public QMainWindow
{
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void initUI();

private slots:
    void updatePositions();

private:
    CustomDockWidget *m_dockListView;
    CustomDockWidget *m_dockConstraint;
    CustomDockWidget *m_dockProperty;
    ControlDialogTreeView *treeView;
};

#endif
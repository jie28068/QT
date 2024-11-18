#ifndef TABLEWIDGET_H
#define TABLEWIDGET_H

#include <QMainWindow>
#include <QListWidgetItem>
#include <QTabWidget>
#include <QListWidget>

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initUI();

private slots:
    void onCurrentRowChanged(int row);

private:
    QTabWidget *tabWidget;
    QListWidget *listWidget;
};

#endif
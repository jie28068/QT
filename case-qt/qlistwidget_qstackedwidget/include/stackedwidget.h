#ifndef STACKDEWIDGET_H
#define STACKDEWIDGET_H

#include <QListWidget>
#include <QStackedWidget>
#include <QMainWindow>

class MainWindowS : public QMainWindow
{
public:
    explicit MainWindowS(QWidget *parent = 0);
    ~MainWindowS();

    void initUI();

private slots:
    void onCurrentRowChanged(int row);

private:
    QStackedWidget *stackWidget;
    QListWidget *listWidget;
};

#endif
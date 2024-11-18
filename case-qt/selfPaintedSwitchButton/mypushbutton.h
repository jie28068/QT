#ifndef MYPUSHBUTTON_H
#define MYPUSHBUTTON_H

#include <QObject>
#include <QWidget>
#include <QPushButton>

class MyPushbutton: public QPushButton
{
public:
    MyPushbutton(QWidget *parent = nullptr);
};

#endif // MYPUSHBUTTON_H

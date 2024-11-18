#ifndef TESTFORM_H
#define TESTFORM_H

#include <QWidget>
#include <QPushButton>

#include "FramelessWindow.h"

class TestForm : public FramelessWindow
{
    Q_OBJECT

public:
    explicit TestForm(QWidget *parent = 0);
    ~TestForm();
    void initUI();

private:
    QPushButton *pushButton_min;
    QPushButton *pushButton_max;
    QPushButton *pushButton_normal;
};

#endif // TESTFORM_H

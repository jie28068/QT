#ifndef MIANWINDOS_H
#define MIANWINDOS_H
#include "scene.h"
#include <QLabel>
#include <QMainWindow>

class MianWindows : public QMainWindow
{
    Q_OBJECT

public:
    MianWindows(Scene *scene, QWidget *parent = nullptr);
    ~MianWindows();
    void initUI();
    void onLabel();

private:
    Scene *m_scene;
    int number;
    QLabel *label2;
};

#endif
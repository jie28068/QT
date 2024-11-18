#ifndef A_H
#define A_H
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QSlider>
#include <QPushButton>
#include "mybattery.h"
#include <QTimer>

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    void initUI();
private slots:
    void on_horizontalSlider_valueChanged(int value);
    void on_pushButton_chiled();
    void timer_update();

private:
    QVBoxLayout *verticalLayout;
    QmyBattery *battery;
    QSlider *horizontalSlider;
    QLabel *LabInfo;
    QPushButton *pushButton;
    QTimer *m_timer;
    int m_value;
};
#endif
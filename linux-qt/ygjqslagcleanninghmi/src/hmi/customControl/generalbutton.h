#ifndef GENERALBUTTON_H
#define GENERALBUTTON_H

#include <QWidget>
#include <QPushButton>

class GeneralButton : public QPushButton
{
    Q_OBJECT

public:
    explicit GeneralButton(QWidget *parent = nullptr);
    explicit GeneralButton(const QString &text, QWidget *parent = nullptr);

private:
    void initUI();
};
#endif
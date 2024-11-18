// SwitchButton.h
#ifndef SWITCHBUTTON_H
#define SWITCHBUTTON_H

#include <QObject>
#include <QWidget>
#include <QPushButton>

class SwitchButton : public QPushButton
{
    Q_OBJECT

public:
    explicit SwitchButton(QWidget *parent = nullptr);

signals:
    void modeChanged(const QString &mode);

public slots:
    void toggleMode();

private:
    bool m_isAutoMode;
};

#endif // SWITCHBUTTON_H


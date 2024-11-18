#ifndef CONTROLTOOLBUTTON_H
#define CONTROLTOOLBUTTON_H

#include <QWidget>
#include <QSize>

class ControlToolButtons : public QWidget
{
    Q_OBJECT
public:
    explicit ControlToolButtons(QString text, QWidget *parnt = 0);
    ~ControlToolButtons();

protected:
    void paintEvent(QPaintEvent *event) override;
    QSize sizeHint() const override;

private:
    QString m_text;
    bool m_isPressed;
    bool m_isHovered;
};

#endif
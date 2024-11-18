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
    bool getHovered();

signals:
    void clicked();

protected:
    void paintEvent(QPaintEvent *event) override;
    QSize sizeHint() const override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    QString m_text;
    bool m_isPressed;
    bool m_isHovered;
};

#endif
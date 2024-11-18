#pragma once

#include <QLabel>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QWidget>
#include <QColor>

class ClickableLabel : public QLabel
{
    Q_OBJECT

public:
    enum Status
    {
        Normal,
        Warning,
        Error
    };

    explicit ClickableLabel(QWidget *parent = nullptr);

    void setStatus(Status status);

protected:
    void mousePressEvent(QMouseEvent *event) override;

signals:
    void clicked();
};
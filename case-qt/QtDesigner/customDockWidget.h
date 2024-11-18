#ifndef CUSTOMDOCKWIDGET_H
#define CUSTOMDOCKWIDGET_H

#include <QDockWidget>
#include <QStackedWidget>
#include <QTabWidget>
#include <QDragEnterEvent>
class CustomDockWidget : public QDockWidget
{
    Q_OBJECT
public:
    explicit CustomDockWidget(const QString &title, QWidget *parent = nullptr,
                              Qt::WindowFlags flags = Qt::WindowFlags());
signals:

public slots:
};
#endif
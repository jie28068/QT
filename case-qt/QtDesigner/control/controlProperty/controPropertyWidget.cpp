#include "controPropertyWidget.h"
#include "objectcontroller.h"

#include <QVBoxLayout>
#include <QLineEdit>
#include <QApplication>
#include <QDesktopWidget>

#include "ControlOneToMany.h"
ControPropertyWidget::ControPropertyWidget(QWidget *parent) : QWidget(parent)
{
    auto theController = new ObjectController(this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(theController);

    // QObject *newObject = new QLineEdit();
    QObject *newObject = new ControlOneToMany();

    // if (!newObject)
    //     return;
    // QWidget *newWidget = qobject_cast<QWidget *>(newObject);
    // if (newWidget)
    // {
    //     QRect r = newWidget->geometry();
    //     r.setSize(newWidget->sizeHint());
    //     r.setWidth(qMax(r.width(), 150));
    //     r.setHeight(qMax(r.height(), 50));
    //     r.moveCenter(QApplication::desktop()->geometry().center());
    //     newWidget->setGeometry(r);
    //     newWidget->show();
    // }

    theController->setObject(newObject);
}

ControPropertyWidget::~ControPropertyWidget()
{
}

#ifndef CUSTOMDIALOG_H
#define CUSTOMDIALOG_H

#include "generalbutton.h"
#include "globals.h"

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QPixmap>
#include <QPalette>

class CustomDialog : public QDialog
{
    Q_OBJECT

public:
    CustomDialog(const QString &message, globals::DialogType type, QWidget *parent = nullptr);
};
#endif
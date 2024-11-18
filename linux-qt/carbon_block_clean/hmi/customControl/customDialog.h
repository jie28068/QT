#ifndef CUSTOMDIALOG_H
#define CUSTOMDIALOG_H

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
    /// @brief 自定义弹窗
    /// @param message 信息
    /// @param type 0 - 提示，1 - 警告
    /// @param parent
    CustomDialog(const QString &message, int type = 0, QWidget *parent = nullptr);
};
#endif
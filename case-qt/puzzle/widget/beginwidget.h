/*
 * @Author: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @Date: 2024-01-06 12:34:08
 * @LastEditors: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @LastEditTime: 2024-01-16 15:02:15
 * @FilePath: \puzzle\widget\beginwidget.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef BEGINWIDGET_H
#define BEGINWIDGET_H

#include "../include/Global.h"

#include <QWidget>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QColor>
#include <QDialog>
#include <QLabel>
#include <QComboBox>
#include <QFileDialog>
#include <QTime>

class LeftMessageBox;
class RightDialog;

using namespace MacroDf;
class Beginwidget : public QDialog
{
    Q_OBJECT
public:
    Beginwidget(QWidget *parent = nullptr);
    ~Beginwidget();

signals:
    void beginGame();

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void initUI();

private:
    /// @brief 左侧界面
    QWidget *leftPanel;
    /// @brief 右侧界面
    QWidget *rightPanel;

    LeftMessageBox *box;
    RightDialog *dia;
};

class LeftMessageBox : public QMessageBox
{
    Q_OBJECT
public:
    LeftMessageBox(MacroDf::Difficulty diff, QWidget *parent = nullptr);
    ~LeftMessageBox();
    void initVariable();
};

class RightDialog : public QDialog
{
    Q_OBJECT
public:
    RightDialog(QWidget *parent = nullptr);

private slots:
    void onPushbutton();
    void onComboBox(const QString &text);

private:
    QLabel *imageLabel;
};
#endif

// QMovie *movie = new QMovie(":/path/to/your/animated.gif");
// label.setMovie(movie);
// label.setScaledContents(true); // 如果你需要GIF图片填充整个QLabel
// movie->start();
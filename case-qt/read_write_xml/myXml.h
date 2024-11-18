#ifndef MYXML_H
#define MYXML_H

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QSplitter>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QDialog>

#include <QDebug>
#include <QFile>
#include <QXmlStreamReader>

class BoardSettingWidget : public QDialog
{
    Q_OBJECT
public:
    BoardSettingWidget(QWidget *p = nullptr);
    ~BoardSettingWidget();

    void readCurrentXmlContent();

    void wirteCurrentXmlContent();

private:
    void initUI();

    QCheckBox *checkBoxEnable; // 启用
    QCheckBox *checkBoxCustom; // 自定义时间
    QLineEdit *lineEdit;
    QComboBox *comboBox;
};

#endif
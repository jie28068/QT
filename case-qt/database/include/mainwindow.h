#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QString>
#include <QSplitter>
#include <QLayout>
#include <QtSql>
#include <QRadioButton>
#include <QSpinBox>
#include <QTableView>
#include <QGroupBox>
#include <QDataWidgetMapper>
#include <QPlainTextEdit>
#include <QDateEdit>

#include "qwcomboboxdelegate.h"

class QWComboBoxDelegate;

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    QSqlDatabase DB; // 数据库连接

    QSqlTableModel *tabModel; // 数据模型

    QItemSelectionModel *theSelection; // 选择模型

    QDataWidgetMapper *dataMapper; // 数据映射

    QWComboBoxDelegate delegateSex;    // 自定义数据代理，性别
    QWComboBoxDelegate delegateDepart; // 自定义数据代理，部门

    void openTable();     // 打开数据表
    void getFieldNames(); // 获取字段名称,填充“排序字段”的comboBox
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void IintUI();
    void initConnects();

private slots:
    void on_currentChanged(const QModelIndex &current, const QModelIndex &previous);

    // QTableView的SelectionModel的行发生了变化，进行处理
    void on_currentRowChanged(const QModelIndex &current, const QModelIndex &previous);

    ///////////////////////
    void on_actOpenDB_triggered();

    void on_actRecAppend_triggered();

    void on_actRecInsert_triggered();

    void on_actRevert_triggered();

    void on_actSubmit_triggered();

    void on_actRecDelete_triggered();

    void on_actPhoto_triggered();

    void on_actPhotoClear_triggered();

    void on_radioBtnAscend_clicked();

    void on_radioBtnDescend_clicked();

    void on_radioBtnMan_clicked();

    void on_radioBtnWoman_clicked();

    void on_radioBtnBoth_clicked();

    void on_comboFields_currentIndexChanged(int index);

    void on_actScan_triggered();

    void on_actOutputDB_triggered();

    void on_initDB_triggered();

    void on_reftext_changed(const QString &text);

private:
    QAction *actOpenDB;
    QAction *actOutputDB;
    QAction *actinitDB;
    QAction *actQuit;
    QAction *actRecAppend;
    QAction *actRecInsert;
    QAction *actSubmit;
    QAction *actRevert;
    QAction *actRecDelete;
    QAction *actPhoto;
    QAction *actPhotoClear;
    QAction *actScan;
    QWidget *centralWidget;
    QSplitter *splitter;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBoxSort;
    QGridLayout *gridLayout_3;
    QLabel *label_14;
    QComboBox *comboFields;
    QComboBox *comborefs;
    QSpacerItem *horizontalSpacer;
    QRadioButton *radioBtnAscend;
    QRadioButton *radioBtnDescend;
    QGroupBox *groupBoxFilter;
    QGridLayout *gridLayout_2;
    QRadioButton *radioBtnMan;
    QRadioButton *radioBtnWoman;
    QRadioButton *radioBtnBoth;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *horizontalSpacer_3;
    QTableView *tableView;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QLabel *label;
    QSpinBox *dbSpinEmpNo;
    QLabel *label_2;
    QLineEdit *dbEditName;
    QLabel *label_3;
    QComboBox *dbComboSex;
    QLabel *label_4;
    QDoubleSpinBox *dbSpinHeight;
    QLabel *label_5;
    QDateEdit *dbEditBirth;
    QLabel *label_10;
    QLineEdit *dbEditMobile;
    QLabel *label_7;
    QComboBox *dbComboProvince;
    QLabel *label_8;
    QLineEdit *dbEditCity;
    QLabel *label_6;
    QComboBox *dbComboDep;
    QPlainTextEdit *dbEditMemo;
    QLabel *label_9;
    QLabel *label_11;
    QSpinBox *dbSpinSalary;
    QLabel *label_12;
    QComboBox *dbComboEdu;
    QVBoxLayout *verticalLayout;
    QLabel *label_13;
    QLabel *dbLabPhoto;
    QSpacerItem *verticalSpacer;
    QMenuBar *menuBar;
    QStatusBar *statusBar;
    QToolBar *mainToolBar;
    QLineEdit *reflineEdit;
};

#endif
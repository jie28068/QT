#ifndef MAINWINDOWS_H
#define MAINWINDOWS_H

#include <QMainWindow>
#include <QListWidgetItem>
#include <QSplitter>
#include <QToolBox>
#include <QGridLayout>
#include <QToolButton>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QAction>
#include <QLineEdit>
#include <QToolBar>
#include <QString>
#include <QCoreApplication>
#include <QMenu>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    void initUI();
    ~MainWindow();
    void retranslateUi();
    // 为QToolButton按钮设置Action
    void setActionsForButton();
    void createSelectionPopMenu();

public slots:
    void on_actListClear_triggered(); // 清除项

    void on_actListIni_triggered(); // 项初始化

    void on_chkBoxListEditable_clicked(bool checked); // chkBoxListEditable单击事件

    // 当前选择项发生变化
    void on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

    void on_actListInsert_triggered(); // 插入项

    void on_actListAppend_triggered(); // 添加项

    void on_actListDelete_triggered(); // 删除当前项

    void on_listWidget_customContextMenuRequested(const QPoint &pos); // 弹出菜单

    void on_toolBox_currentChanged(int index); // ToolBox当前组变化时，显示TabWidget相应的页面

    void on_actSelALL_triggered(); // 全选

    void on_actSelNone_triggered(); // 全不选

    void on_actSelInvs_triggered(); // 反选
private:
    QAction *actListIni;    // 初始化
    QAction *actListClear;  // 清除
    QAction *actListInsert; // 插入
    QAction *actListAppend; // 添加
    QAction *actListDelete; // 删除
    QAction *actSelALL;     // 全选
    QAction *actSelNone;    // 全不选
    QAction *actSelInvs;    // 反选
    QAction *actQuit;       // 离开
    QAction *actSelPopMenu;
    QWidget *centralWidget;
    QSplitter *splitter;
    QToolBox *toolBox;
    QWidget *page;
    QGridLayout *gridLayout;
    QToolButton *tBtnListIni;
    QToolButton *tBtnListClear;
    QToolButton *tBtnListInsert;
    QToolButton *tBtnListAppend;
    QToolButton *tBtnListDelete;
    QWidget *page_2;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_2;
    QLineEdit *lineEdit;
    QSpinBox *spinBox;
    QSpacerItem *verticalSpacer;
    QWidget *page_3;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *editCutItemText;
    QCheckBox *chkBoxListEditable;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_2;
    QToolButton *tBtnSelectItem;
    QToolButton *tBtnSelALL;
    QToolButton *tBtnSelNone;
    QToolButton *tBtnSelInvs;
    QListWidget *listWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout_3;
    QWidget *tab_2;
    QToolBar *mainToolBar;
};
#endif
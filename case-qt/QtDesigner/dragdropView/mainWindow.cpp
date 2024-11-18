
#include "controlTool.h"
#include "customDockWidget.h"
#include "mainWindow.h"
#include "globalDefinition.h"
#include "controlDialogTreeView.h"

#include <QStatusBar>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    initUI();
    setFixedSize(1200, 600);
}

MainWindow::~MainWindow()
{
}

void MainWindow::initUI()
{
    ControlTools *w = new ControlTools(this);
    w->addPage(GlobalDefinition::controlParameters);
    w->addPage(GlobalDefinition::controlContainer);
    w->addPage(GlobalDefinition::controlDisplay);
    w->addPage(GlobalDefinition::controlOperation);

    // 顶层菜单
    QToolBar *toolBar = new QToolBar(this);
    toolBar->addWidget(new QPushButton(QString::fromUtf8("预览")));
    this->addToolBar(Qt::TopToolBarArea, toolBar);
    // 左停靠窗口
    m_dockListView = new CustomDockWidget(QString::fromUtf8("控件"), this);
    m_dockListView->setWidget(w);
    this->addDockWidget(Qt::LeftDockWidgetArea, m_dockListView);

    m_dockConstraint = new CustomDockWidget(QString::fromUtf8("约束"), this);
    m_dockConstraint->setWidget(new QTextEdit(QString::fromUtf8("约束窗口")));
    this->addDockWidget(Qt::LeftDockWidgetArea, m_dockConstraint);
    m_dockConstraint->setVisible(false);

    // 中间
    QWidget *centerWidget = new QWidget(this);
    QTabWidget *tabWidget = new QTabWidget(centerWidget);
    // 创建页面
    QWidget *page1 = new QWidget(tabWidget);
    QWidget *page2 = new QWidget(tabWidget);
    // 为每个页面添加一些内容
    treeView = new ControlDialogTreeView(page1);
    QVBoxLayout *page1Layout = new QVBoxLayout(page1);
    page1Layout->addWidget(treeView);
    page1->setLayout(page1Layout);

    QLabel *label2 = new QLabel("33333", page2);
    // 将页面添加到QTabWidget
    tabWidget->addTab(page1, "参数与对话框");
    tabWidget->addTab(page2, "图标");
    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(tabWidget);
    centerWidget->setLayout(layout);
    this->setCentralWidget(centerWidget);
    // end
    // 右停靠窗口
    m_dockProperty = new CustomDockWidget(QString::fromUtf8("属性编辑器"), this);

    this->addDockWidget(Qt::RightDockWidgetArea, m_dockProperty);

    connect(tabWidget, &QTabWidget::currentChanged, this, [=](int index)
            {
        if (index == 0)
        {
            m_dockListView->setVisible(true);
            m_dockConstraint->setVisible(false);
            m_dockProperty->setVisible(true);
        }
        else
        {
            m_dockListView->setVisible(false);
            m_dockConstraint->setVisible(true);
            m_dockProperty->setVisible(false);
        } });

    connect(treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updatePositions);
}

void MainWindow::updatePositions()
{
    const int row = treeView->selectionModel()->currentIndex().row();
    const int column = treeView->selectionModel()->currentIndex().column();
    if (treeView->selectionModel()->currentIndex().parent().isValid())
        statusBar()->showMessage(tr("Position: (%1,%2)").arg(row).arg(column));
    else
        statusBar()->showMessage(tr("Position: (%1,%2) in top level").arg(row).arg(column));
}
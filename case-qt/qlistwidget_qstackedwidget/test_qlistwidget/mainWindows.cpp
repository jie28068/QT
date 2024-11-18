#include "mainWindows.h"

MainWindow::MainWindow(QWidget *parent)
{
    initUI();
    setCentralWidget(this->splitter); // 设置中心组件
    setActionsForButton();
    createSelectionPopMenu();

    connect(actListClear, &QAction::triggered, this, &MainWindow::on_actListClear_triggered);
    connect(actListIni, &QAction::triggered, this, &MainWindow::on_actListIni_triggered);
    connect(chkBoxListEditable, &QCheckBox::stateChanged, this, &MainWindow::on_chkBoxListEditable_clicked);
    connect(listWidget, &QListWidget::currentItemChanged, this, &MainWindow::on_listWidget_currentItemChanged);
    connect(actListInsert, &QAction::triggered, this, &MainWindow::on_actListInsert_triggered);
    connect(actListAppend, &QAction::triggered, this, &MainWindow::on_actListAppend_triggered);
    connect(actListDelete, &QAction::triggered, this, &MainWindow::on_actListDelete_triggered);
    connect(toolBox, &QToolBox::currentChanged, this, &MainWindow::on_toolBox_currentChanged);
    connect(actSelALL, &QAction::triggered, this, &MainWindow::on_actSelALL_triggered);
    connect(actSelNone, &QAction::triggered, this, &MainWindow::on_actSelNone_triggered);
    connect(actSelInvs, &QAction::triggered, this, &MainWindow::on_actSelInvs_triggered);
    // connect(listWidget, &QListWidget::triggered, this, &MainWindow::on_listWidget_customContextMenuRequested);
}

void MainWindow::initUI()
{
    resize(600, 400);
    QFont font;
    font.setPointSize(10);
    setFont(font);
    actListIni = new QAction(this);
    actListIni->setObjectName(QString::fromUtf8("actListIni"));
    QIcon icon;
    icon.addFile(QString::fromUtf8(":/images/icons/128.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actListIni->setIcon(icon);
    actListClear = new QAction(this);
    actListClear->setObjectName(QString::fromUtf8("actListClear"));
    QIcon icon1;
    icon1.addFile(QString::fromUtf8(":/images/icons/delete1.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actListClear->setIcon(icon1);
    actListInsert = new QAction(this);
    actListInsert->setObjectName(QString::fromUtf8("actListInsert"));
    QIcon icon2;
    icon2.addFile(QString::fromUtf8(":/images/icons/424.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actListInsert->setIcon(icon2);
    actListAppend = new QAction(this);
    actListAppend->setObjectName(QString::fromUtf8("actListAppend"));
    QIcon icon3;
    icon3.addFile(QString::fromUtf8(":/images/icons/316.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actListAppend->setIcon(icon3);
    actListDelete = new QAction(this);
    actListDelete->setObjectName(QString::fromUtf8("actListDelete"));
    QIcon icon4;
    icon4.addFile(QString::fromUtf8(":/images/icons/324.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actListDelete->setIcon(icon4);
    actSelALL = new QAction(this);
    actSelALL->setObjectName(QString::fromUtf8("actSelALL"));
    actSelNone = new QAction(this);
    actSelNone->setObjectName(QString::fromUtf8("actSelNone"));
    actSelNone->setMenuRole(QAction::AboutRole);
    actSelInvs = new QAction(this);
    actSelInvs->setObjectName(QString::fromUtf8("actSelInvs"));
    actQuit = new QAction(this);
    actQuit->setObjectName(QString::fromUtf8("actQuit"));
    QIcon icon5;
    icon5.addFile(QString::fromUtf8(":/images/icons/exit.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actQuit->setIcon(icon5);
    actSelPopMenu = new QAction(this);
    actSelPopMenu->setObjectName(QString::fromUtf8("actSelPopMenu"));
    QIcon icon6;
    icon6.addFile(QString::fromUtf8(":/images/icons/124.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actSelPopMenu->setIcon(icon6);

    centralWidget = new QWidget(this);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));

    splitter = new QSplitter(centralWidget);
    splitter->setObjectName(QString::fromUtf8("splitter"));
    splitter->setGeometry(QRect(5, 5, 556, 271));
    splitter->setFrameShape(QFrame::Box);
    splitter->setFrameShadow(QFrame::Plain);
    splitter->setLineWidth(1);
    splitter->setMidLineWidth(3);
    splitter->setOrientation(Qt::Horizontal);
    splitter->setOpaqueResize(true);

    toolBox = new QToolBox(splitter);
    toolBox->setObjectName(QString::fromUtf8("toolBox"));
    toolBox->setMinimumSize(QSize(150, 0));
    toolBox->setMaximumSize(QSize(300, 16777215));

    page = new QWidget();
    page->setObjectName(QString::fromUtf8("page"));
    page->setGeometry(QRect(0, 0, 150, 176));

    gridLayout = new QGridLayout(page);
    gridLayout->setSpacing(6);
    gridLayout->setContentsMargins(11, 11, 11, 11);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

    tBtnListIni = new QToolButton(page);
    tBtnListIni->setObjectName(QString::fromUtf8("tBtnListIni"));
    tBtnListIni->setMinimumSize(QSize(120, 25));
    tBtnListIni->setPopupMode(QToolButton::DelayedPopup);
    tBtnListIni->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    tBtnListIni->setAutoRaise(false);
    tBtnListIni->setArrowType(Qt::NoArrow);
    gridLayout->addWidget(tBtnListIni, 0, 0, 1, 1);

    tBtnListClear = new QToolButton(page);
    tBtnListClear->setObjectName(QString::fromUtf8("tBtnListClear"));
    tBtnListClear->setMinimumSize(QSize(120, 25));
    tBtnListClear->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    gridLayout->addWidget(tBtnListClear, 1, 0, 1, 1);

    tBtnListInsert = new QToolButton(page);
    tBtnListInsert->setObjectName(QString::fromUtf8("tBtnListInsert"));
    tBtnListInsert->setMinimumSize(QSize(120, 25));
    tBtnListInsert->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    gridLayout->addWidget(tBtnListInsert, 2, 0, 1, 1);

    tBtnListAppend = new QToolButton(page);
    tBtnListAppend->setObjectName(QString::fromUtf8("tBtnListAppend"));
    tBtnListAppend->setMinimumSize(QSize(120, 25));
    tBtnListAppend->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    gridLayout->addWidget(tBtnListAppend, 3, 0, 1, 1);

    tBtnListDelete = new QToolButton(page);
    tBtnListDelete->setObjectName(QString::fromUtf8("tBtnListDelete"));
    tBtnListDelete->setMinimumSize(QSize(120, 25));
    tBtnListDelete->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    gridLayout->addWidget(tBtnListDelete, 4, 0, 1, 1);

    QIcon icon7;
    icon7.addFile(QString::fromUtf8(":/images/icons/410.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    toolBox->addItem(page, icon7, QString::fromUtf8("QListWidget\346\223\215\344\275\234"));
    page_2 = new QWidget();
    page_2->setObjectName(QString::fromUtf8("page_2"));
    page_2->setGeometry(QRect(0, 0, 150, 176));
    verticalLayout_4 = new QVBoxLayout(page_2);
    verticalLayout_4->setSpacing(6);
    verticalLayout_4->setContentsMargins(11, 11, 11, 11);
    verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
    groupBox_4 = new QGroupBox(page_2);
    groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
    verticalLayout_3 = new QVBoxLayout(groupBox_4);
    verticalLayout_3->setSpacing(6);
    verticalLayout_3->setContentsMargins(11, 11, 11, 11);
    verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
    label_2 = new QLabel(groupBox_4);
    label_2->setObjectName(QString::fromUtf8("label_2"));

    verticalLayout_3->addWidget(label_2);

    lineEdit = new QLineEdit(groupBox_4);
    lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

    verticalLayout_3->addWidget(lineEdit);

    spinBox = new QSpinBox(groupBox_4);
    spinBox->setObjectName(QString::fromUtf8("spinBox"));

    verticalLayout_3->addWidget(spinBox);

    verticalLayout_4->addWidget(groupBox_4);

    verticalSpacer = new QSpacerItem(20, 119, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout_4->addItem(verticalSpacer);

    QIcon icon8;
    icon8.addFile(QString::fromUtf8(":/images/icons/408.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    toolBox->addItem(page_2, icon8, QString::fromUtf8("QTreeWidget\346\223\215\344\275\234"));
    page_3 = new QWidget();
    page_3->setObjectName(QString::fromUtf8("page_3"));
    page_3->setGeometry(QRect(0, 0, 150, 176));
    QIcon icon9;
    icon9.addFile(QString::fromUtf8(":/images/icons/412.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    toolBox->addItem(page_3, icon9, QString::fromUtf8("QTableWidget\346\223\215\344\275\234"));
    splitter->addWidget(toolBox);
    tabWidget = new QTabWidget(splitter);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    tabWidget->setTabPosition(QTabWidget::North);
    tabWidget->setTabShape(QTabWidget::Rounded);
    tabWidget->setElideMode(Qt::ElideLeft);
    tab_3 = new QWidget();
    tab_3->setObjectName(QString::fromUtf8("tab_3"));
    verticalLayout = new QVBoxLayout(tab_3);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    verticalLayout->setContentsMargins(4, 4, 4, 4);
    groupBox = new QGroupBox(tab_3);
    groupBox->setObjectName(QString::fromUtf8("groupBox"));
    horizontalLayout = new QHBoxLayout(groupBox);
    horizontalLayout->setSpacing(10);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalLayout->setContentsMargins(-1, 2, -1, 2);
    label = new QLabel(groupBox);
    label->setObjectName(QString::fromUtf8("label"));

    horizontalLayout->addWidget(label);

    editCutItemText = new QLineEdit(groupBox);
    editCutItemText->setObjectName(QString::fromUtf8("editCutItemText"));
    editCutItemText->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 255);"));
    editCutItemText->setClearButtonEnabled(false);

    horizontalLayout->addWidget(editCutItemText);

    chkBoxListEditable = new QCheckBox(groupBox);
    chkBoxListEditable->setObjectName(QString::fromUtf8("chkBoxListEditable"));

    horizontalLayout->addWidget(chkBoxListEditable);

    verticalLayout->addWidget(groupBox);

    groupBox_2 = new QGroupBox(tab_3);
    groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
    horizontalLayout_2 = new QHBoxLayout(groupBox_2);
    horizontalLayout_2->setSpacing(6);
    horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
    horizontalLayout_2->setContentsMargins(-1, 2, -1, 2);
    tBtnSelectItem = new QToolButton(groupBox_2);
    tBtnSelectItem->setObjectName(QString::fromUtf8("tBtnSelectItem"));
    tBtnSelectItem->setMinimumSize(QSize(100, 25));
    tBtnSelectItem->setPopupMode(QToolButton::MenuButtonPopup);
    tBtnSelectItem->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

    horizontalLayout_2->addWidget(tBtnSelectItem);

    tBtnSelALL = new QToolButton(groupBox_2);
    tBtnSelALL->setObjectName(QString::fromUtf8("tBtnSelALL"));
    tBtnSelALL->setMinimumSize(QSize(70, 25));

    horizontalLayout_2->addWidget(tBtnSelALL);

    tBtnSelNone = new QToolButton(groupBox_2);
    tBtnSelNone->setObjectName(QString::fromUtf8("tBtnSelNone"));
    tBtnSelNone->setMinimumSize(QSize(70, 25));

    horizontalLayout_2->addWidget(tBtnSelNone);

    tBtnSelInvs = new QToolButton(groupBox_2);
    tBtnSelInvs->setObjectName(QString::fromUtf8("tBtnSelInvs"));
    tBtnSelInvs->setMinimumSize(QSize(70, 25));

    horizontalLayout_2->addWidget(tBtnSelInvs);

    verticalLayout->addWidget(groupBox_2);

    listWidget = new QListWidget(tab_3);
    QIcon icon10;
    icon10.addFile(QString::fromUtf8(":/images/icons/check2.ico"), QSize(), QIcon::Normal, QIcon::Off);
    QListWidgetItem *__qlistwidgetitem = new QListWidgetItem(listWidget);
    __qlistwidgetitem->setCheckState(Qt::Checked);
    __qlistwidgetitem->setIcon(icon10);
    __qlistwidgetitem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsDragEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    QListWidgetItem *__qlistwidgetitem1 = new QListWidgetItem(listWidget);
    __qlistwidgetitem1->setCheckState(Qt::Unchecked);
    __qlistwidgetitem1->setIcon(icon10);
    __qlistwidgetitem1->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    listWidget->setObjectName(QString::fromUtf8("listWidget"));
    listWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    listWidget->setLayoutMode(QListView::SinglePass);
    listWidget->setViewMode(QListView::ListMode);
    listWidget->setSelectionRectVisible(false);

    verticalLayout->addWidget(listWidget);

    tabWidget->addTab(tab_3, QString());
    tab = new QWidget();
    tab->setObjectName(QString::fromUtf8("tab"));
    verticalLayout_2 = new QVBoxLayout(tab);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(11, 11, 11, 11);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    groupBox_3 = new QGroupBox(tab);
    groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
    horizontalLayout_3 = new QHBoxLayout(groupBox_3);
    horizontalLayout_3->setSpacing(6);
    horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));

    verticalLayout_2->addWidget(groupBox_3);

    tabWidget->addTab(tab, QString());
    tab_2 = new QWidget();
    tab_2->setObjectName(QString::fromUtf8("tab_2"));
    tabWidget->addTab(tab_2, QString());
    splitter->addWidget(tabWidget);
    setCentralWidget(centralWidget);
    mainToolBar = new QToolBar(this);
    mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
    mainToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    this->addToolBar(Qt::TopToolBarArea, mainToolBar);

    mainToolBar->addAction(actListIni);
    mainToolBar->addAction(actListClear);
    mainToolBar->addAction(actListInsert);
    mainToolBar->addAction(actListAppend);
    mainToolBar->addAction(actListDelete);
    mainToolBar->addSeparator();

    retranslateUi();

    QObject::connect(actQuit, SIGNAL(triggered()), this, SLOT(close()));
    QObject::connect(actSelPopMenu, SIGNAL(triggered()), actSelInvs, SLOT(trigger()));

    toolBox->setCurrentIndex(0);
    tabWidget->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
}

void MainWindow::retranslateUi()
{
    this->setWindowTitle(QCoreApplication::translate("MainWindow", "QListWidget\347\232\204\344\275\277\347\224\250", nullptr));
    actListIni->setText(QCoreApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\345\210\227\350\241\250", nullptr));
#if QT_CONFIG(tooltip)
    actListIni->setToolTip(QCoreApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\345\210\227\350\241\250", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actListIni->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+I", nullptr));
#endif // QT_CONFIG(shortcut)
    actListClear->setText(QCoreApplication::translate("MainWindow", "\346\270\205\351\231\244\345\210\227\350\241\250", nullptr));
#if QT_CONFIG(tooltip)
    actListClear->setToolTip(QCoreApplication::translate("MainWindow", "\346\270\205\351\231\244\345\210\227\350\241\250", nullptr));
#endif // QT_CONFIG(tooltip)
    actListInsert->setText(QCoreApplication::translate("MainWindow", "\346\217\222\345\205\245\351\241\271", nullptr));
#if QT_CONFIG(tooltip)
    actListInsert->setToolTip(QCoreApplication::translate("MainWindow", "\346\217\222\345\205\245\351\241\271", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actListInsert->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+S", nullptr));
#endif // QT_CONFIG(shortcut)
    actListAppend->setText(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\351\241\271", nullptr));
#if QT_CONFIG(tooltip)
    actListAppend->setToolTip(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\351\241\271", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actListAppend->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+A", nullptr));
#endif // QT_CONFIG(shortcut)
    actListDelete->setText(QCoreApplication::translate("MainWindow", "\345\210\240\351\231\244\345\275\223\345\211\215\351\241\271", nullptr));
#if QT_CONFIG(tooltip)
    actListDelete->setToolTip(QCoreApplication::translate("MainWindow", "\345\210\240\351\231\244\345\275\223\345\211\215\351\241\271", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actListDelete->setShortcut(QCoreApplication::translate("MainWindow", "Del", nullptr));
#endif // QT_CONFIG(shortcut)
    actSelALL->setText(QCoreApplication::translate("MainWindow", "\345\205\250\351\200\211", nullptr));
#if QT_CONFIG(tooltip)
    actSelALL->setToolTip(QCoreApplication::translate("MainWindow", "\345\205\250\351\200\211", nullptr));
#endif // QT_CONFIG(tooltip)
    actSelNone->setText(QCoreApplication::translate("MainWindow", "\345\205\250\344\270\215\351\200\211", nullptr));
#if QT_CONFIG(tooltip)
    actSelNone->setToolTip(QCoreApplication::translate("MainWindow", "\345\205\250\344\270\215\351\200\211", nullptr));
#endif // QT_CONFIG(tooltip)
    actSelInvs->setText(QCoreApplication::translate("MainWindow", "\345\217\215\351\200\211", nullptr));
#if QT_CONFIG(tooltip)
    actSelInvs->setToolTip(QCoreApplication::translate("MainWindow", "\345\217\215\351\200\211", nullptr));
#endif // QT_CONFIG(tooltip)
    actQuit->setText(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272", nullptr));
#if QT_CONFIG(tooltip)
    actQuit->setToolTip(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272\347\250\213\345\272\217", nullptr));
#endif // QT_CONFIG(tooltip)
    actSelPopMenu->setText(QCoreApplication::translate("MainWindow", "\351\241\271\351\200\211\346\213\251", nullptr));
#if QT_CONFIG(tooltip)
    actSelPopMenu->setToolTip(QCoreApplication::translate("MainWindow", "\351\241\271\351\200\211\346\213\251", nullptr));
#endif // QT_CONFIG(tooltip)
    tBtnListIni->setText(QCoreApplication::translate("MainWindow", "tBtnListIni", nullptr));
    tBtnListClear->setText(QCoreApplication::translate("MainWindow", "tBtnListClear", nullptr));
    tBtnListInsert->setText(QCoreApplication::translate("MainWindow", "tBtnListInsert", nullptr));
    tBtnListAppend->setText(QCoreApplication::translate("MainWindow", "tBtnListAppend", nullptr));
    tBtnListDelete->setText(QCoreApplication::translate("MainWindow", "tBtnListDelete", nullptr));
    toolBox->setItemText(toolBox->indexOf(page), QCoreApplication::translate("MainWindow", "QListWidget\346\223\215\344\275\234", nullptr));
    groupBox_4->setTitle(QCoreApplication::translate("MainWindow", "GroupBox", nullptr));
    label_2->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
    toolBox->setItemText(toolBox->indexOf(page_2), QCoreApplication::translate("MainWindow", "QTreeWidget\346\223\215\344\275\234", nullptr));
    toolBox->setItemText(toolBox->indexOf(page_3), QCoreApplication::translate("MainWindow", "QTableWidget\346\223\215\344\275\234", nullptr));
    groupBox->setTitle(QString());
    label->setText(QCoreApplication::translate("MainWindow", "\345\275\223\345\211\215\351\241\271\345\217\230\345\214\226", nullptr));
    chkBoxListEditable->setText(QCoreApplication::translate("MainWindow", "\345\217\257\347\274\226\350\276\221", nullptr));
    groupBox_2->setTitle(QString());
    tBtnSelectItem->setText(QCoreApplication::translate("MainWindow", "tBtnSelectItem", nullptr));
    tBtnSelALL->setText(QCoreApplication::translate("MainWindow", "tBtnSelALL", nullptr));
    tBtnSelNone->setText(QCoreApplication::translate("MainWindow", "tBtnSelNone", nullptr));
    tBtnSelInvs->setText(QCoreApplication::translate("MainWindow", "tBtnSelInvs", nullptr));

    const bool __sortingEnabled = listWidget->isSortingEnabled();
    listWidget->setSortingEnabled(false);
    QListWidgetItem *___qlistwidgetitem = listWidget->item(0);
    ___qlistwidgetitem->setText(QCoreApplication::translate("MainWindow", "New Item", nullptr));
    QListWidgetItem *___qlistwidgetitem1 = listWidget->item(1);
    ___qlistwidgetitem1->setText(QCoreApplication::translate("MainWindow", "New Item2", nullptr));
    listWidget->setSortingEnabled(__sortingEnabled);

    tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("MainWindow", "QListWidget", nullptr));
    groupBox_3->setTitle(QString());
    tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "QTreeWidget", nullptr));
    tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "QTableWidget", nullptr));
} // retranslateUi

void MainWindow::setActionsForButton()
{
    this->tBtnListIni->setDefaultAction(this->actListIni);
    this->tBtnListClear->setDefaultAction(this->actListClear);
    this->tBtnListInsert->setDefaultAction(this->actListInsert);
    this->tBtnListAppend->setDefaultAction(this->actListAppend);
    this->tBtnListDelete->setDefaultAction(this->actListDelete);

    this->tBtnSelALL->setDefaultAction(this->actSelALL);
    this->tBtnSelNone->setDefaultAction(this->actSelNone);
    this->tBtnSelInvs->setDefaultAction(this->actSelInvs);
}

void MainWindow::createSelectionPopMenu()
{
    // 创建下拉菜单
    QMenu *menuSelection = new QMenu(this); // 创建选择弹出式菜单
    menuSelection->addAction(this->actSelALL);
    menuSelection->addAction(this->actSelNone);
    menuSelection->addAction(this->actSelInvs);

    // listWidget上方的tBtnSelectItem按钮
    this->tBtnSelectItem->setPopupMode(QToolButton::MenuButtonPopup);       // 菜单弹出模式，执行按钮的Action
    this->tBtnSelectItem->setToolButtonStyle(Qt::ToolButtonTextBesideIcon); // 按钮样式
    this->tBtnSelectItem->setDefaultAction(this->actSelPopMenu);            // 关联Action
    this->tBtnSelectItem->setMenu(menuSelection);                           // 设置下拉菜单

    // 工具栏上的 下拉式菜单按钮
    QToolButton *aBtn = new QToolButton(this);
    aBtn->setPopupMode(QToolButton::InstantPopup);         // button's own action is not triggered.
    aBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon); // 按钮样式
    aBtn->setDefaultAction(this->actSelPopMenu);           // 设置Action,获取图标、标题等设置
    aBtn->setMenu(menuSelection);                          // 设置下拉菜单
    this->mainToolBar->addWidget(aBtn);                    // 工具栏添加按钮

    // 工具栏添加分隔条，和“退出”按钮
    this->mainToolBar->addSeparator();
    this->mainToolBar->addAction(this->actQuit);
}

void MainWindow::on_actListIni_triggered()
{
    QListWidgetItem *aItem; // 每一行是一个QListWidgetItem

    QIcon aIcon;
    aIcon.addFile(":/images/icons/check2.ico");       // 设置ICON的图标
    bool chk = this->chkBoxListEditable->isChecked(); // 是否可编辑

    this->listWidget->clear(); // 清除项
    for (int i = 0; i < 10; i++)
    {
        QString str = QString::asprintf("Item %d", i);
        aItem = new QListWidgetItem(); // 新建一个项

        aItem->setText(str);               // 设置文字标签
        aItem->setIcon(aIcon);             // 设置图标
        aItem->setCheckState(Qt::Checked); // 设置为选中状态
        if (chk)                           // 可编辑, 设置flags
            aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        else // 不可编辑, 设置flags
            aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        this->listWidget->addItem(aItem); // 增加一个项
    }
}

void MainWindow::on_actListClear_triggered()
{
    listWidget->clear();
}

void MainWindow::on_chkBoxListEditable_clicked(bool checked)
{ // 可编辑 QCheckBox的响应代码， 设置所有项是否可编辑
    int i, cnt;
    QListWidgetItem *aItem;

    cnt = this->listWidget->count(); // 项的个数
    for (i = 0; i < cnt; i++)
    {
        aItem = this->listWidget->item(i); // 获得一个项
        if (checked)                       // 可编辑
            aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        else
            aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    }
}

void MainWindow::on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{ // listWidget当前选中项发生变化
    QString str;
    if (current != NULL) // 需要检测变量指针是否为空
    {
        if (previous == NULL) // 需要检测变量指针是否为空
            str = tr("Current:") + current->text();
        else
            str = tr("Last:") + previous->text() + tr(";Current:") + current->text();
        this->editCutItemText->setText(str);
    }
}

void MainWindow::on_actListInsert_triggered()
{ // 插入一个项
    QIcon aIcon;
    aIcon.addFile(":/images/icons/check2.ico"); // 图标

    bool chk = this->chkBoxListEditable->isChecked(); // 是否可比那几

    QListWidgetItem *aItem = new QListWidgetItem("New Inserted Item"); // 创建一个项
    aItem->setIcon(aIcon);                                             // 设置图标
    aItem->setCheckState(Qt::Checked);                                 // 设置为checked
    if (chk)                                                           // 设置标记
        aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    else
        aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);

    this->listWidget->insertItem(this->listWidget->currentRow(), aItem); // 在当前行的上方插入一个项
}

void MainWindow::on_actListAppend_triggered()
{ // 增加一个项
    QIcon aIcon;
    aIcon.addFile(":/images/icons/check2.ico"); // 设定图标

    bool chk = this->chkBoxListEditable->isChecked(); // 是否可编辑

    QListWidgetItem *aItem = new QListWidgetItem("New Added Item"); // 创建一个Item
    aItem->setIcon(aIcon);                                          // 设置图标
    aItem->setCheckState(Qt::Checked);                              // 设置为checked
    if (chk)                                                        // 设置标志
        aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    else
        aItem->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);

    this->listWidget->addItem(aItem); // 增加一个项
}

void MainWindow::on_actListDelete_triggered()
{                                             // 删除当前项
    int row = this->listWidget->currentRow(); // 当前行

    QListWidgetItem *aItem = this->listWidget->takeItem(row); // 移除指定行的项，但不delete
    delete aItem;                                             // 需要手工删除对象
}

void MainWindow::on_listWidget_customContextMenuRequested(const QPoint &pos)
{
    Q_UNUSED(pos);
    QMenu *menuList = new QMenu(this); // 创建菜单

    // 添加Actions创建菜单项
    menuList->addAction(this->actListIni);
    menuList->addAction(this->actListClear);
    menuList->addAction(this->actListInsert);
    menuList->addAction(this->actListAppend);
    menuList->addAction(this->actListDelete);
    menuList->addSeparator();
    menuList->addAction(this->actSelALL);
    menuList->addAction(this->actSelNone);
    menuList->addAction(this->actSelInvs);

    menuList->exec(QCursor::pos()); // 在鼠标光标位置显示右键快捷菜单

    delete menuList; // 手工创建的指针必须手工删除
}

void MainWindow::on_toolBox_currentChanged(int index)
{
    this->tabWidget->setCurrentIndex(index); // ToolBox当前页与tabWidget的当前页联动
}

void MainWindow::on_actSelALL_triggered()
{                                        // 项全选
    int cnt = this->listWidget->count(); // 项个数
    for (int i = 0; i < cnt; i++)
    {
        QListWidgetItem *aItem = this->listWidget->item(i); // 获取一个项
        aItem->setCheckState(Qt::Checked);                  // 设置为选中
    }
}

void MainWindow::on_actSelNone_triggered()
{ // 全不选
    int i, cnt;
    QListWidgetItem *aItem;

    cnt = this->listWidget->count(); // 项个数
    for (i = 0; i < cnt; i++)
    {
        aItem = this->listWidget->item(i);   // 获取一个项
        aItem->setCheckState(Qt::Unchecked); // 不选
    }
}

void MainWindow::on_actSelInvs_triggered()
{ // 反选
    int i, cnt;
    QListWidgetItem *aItem;

    cnt = this->listWidget->count(); // 项个数
    for (i = 0; i < cnt; i++)
    {
        aItem = this->listWidget->item(i); // 获取一个项
        if (aItem->checkState() != Qt::Checked)
            aItem->setCheckState(Qt::Checked);
        else
            aItem->setCheckState(Qt::Unchecked);
    }
}
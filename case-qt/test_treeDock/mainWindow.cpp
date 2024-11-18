#include "mainWindow.h"
#include <QScrollArea>
#include <QLabel>
#include <QHBoxLayout>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    initUI();
    QString aa = "天不生我李淳罡，剑道万古如长夜。 ";
    LabFileName = new QLabel(aa);
    statusBar->addWidget(LabFileName);
    setCentralWidget(scrollArea); // 设置中心布局组件
    iniTree();                    // 初始化目录树
}

void MainWindow::initUI()
{
    if (this->objectName().isEmpty())
        this->setObjectName(QString::fromUtf8("MainWindow"));
    this->resize(934, 500);
    QFont font;
    font.setPointSize(10);
    this->setFont(font);
    actAddFolder = new QAction(this);
    actAddFolder->setObjectName(QString::fromUtf8("actAddFolder"));
    QIcon icon;
    icon.addFile(QString::fromUtf8(":images/icon"), QSize(), QIcon::Normal, QIcon::Off);
    actAddFolder->setIcon(icon);
    actAddFiles = new QAction(this);
    actAddFiles->setObjectName(QString::fromUtf8("actAddFiles"));
    QIcon icon1;
    icon1.addFile(QString::fromUtf8(":/images/icon1"), QSize(), QIcon::Normal, QIcon::Off);
    actAddFiles->setIcon(icon1);
    actZoomIn = new QAction(this);
    actZoomIn->setObjectName(QString::fromUtf8("actZoomIn"));
    actZoomIn->setEnabled(false);
    QIcon icon2;
    icon2.addFile(QString::fromUtf8(":images/icons/418.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actZoomIn->setIcon(icon2);
    actZoomOut = new QAction(this);
    actZoomOut->setObjectName(QString::fromUtf8("actZoomOut"));
    actZoomOut->setEnabled(false);
    QIcon icon3;
    icon3.addFile(QString::fromUtf8(":/images/icons/416.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actZoomOut->setIcon(icon3);
    actZoomRealSize = new QAction(this);
    actZoomRealSize->setObjectName(QString::fromUtf8("actZoomRealSize"));
    actZoomRealSize->setEnabled(false);
    QIcon icon4;
    icon4.addFile(QString::fromUtf8(":/images/icons/414.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actZoomRealSize->setIcon(icon4);
    actZoomFitW = new QAction(this);
    actZoomFitW->setObjectName(QString::fromUtf8("actZoomFitW"));
    actZoomFitW->setEnabled(false);
    QIcon icon5;
    icon5.addFile(QString::fromUtf8(":/images/icons/424.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actZoomFitW->setIcon(icon5);
    actDeleteItem = new QAction(this);
    actDeleteItem->setObjectName(QString::fromUtf8("actDeleteItem"));
    actDeleteItem->setEnabled(false);
    QIcon icon6;
    icon6.addFile(QString::fromUtf8(":/images/icons/delete1.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actDeleteItem->setIcon(icon6);
    actQuit = new QAction(this);
    actQuit->setObjectName(QString::fromUtf8("actQuit"));
    QIcon icon7;
    icon7.addFile(QString::fromUtf8(":/images/icons/exit.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actQuit->setIcon(icon7);
    actZoomFitH = new QAction(this);
    actZoomFitH->setObjectName(QString::fromUtf8("actZoomFitH"));
    actZoomFitH->setEnabled(false);
    QIcon icon8;
    icon8.addFile(QString::fromUtf8(":/images/icons/426.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actZoomFitH->setIcon(icon8);
    actScanItems = new QAction(this);
    actScanItems->setObjectName(QString::fromUtf8("actScanItems"));
    QIcon icon9;
    icon9.addFile(QString::fromUtf8(":/images/icons/fold.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actScanItems->setIcon(icon9);
    actDockVisible = new QAction(this);
    actDockVisible->setObjectName(QString::fromUtf8("actDockVisible"));
    actDockVisible->setCheckable(true);
    actDockVisible->setChecked(true);
    actDockVisible->setIcon(icon4);
    actDockFloat = new QAction(this);
    actDockFloat->setObjectName(QString::fromUtf8("actDockFloat"));
    actDockFloat->setCheckable(true);
    actDockFloat->setChecked(false);
    QIcon icon10;
    icon10.addFile(QString::fromUtf8(":/images/icons/814.bmp"), QSize(), QIcon::Normal, QIcon::Off);
    actDockFloat->setIcon(icon10);

    centralWidget = new QWidget(this);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    scrollArea = new QScrollArea(centralWidget);
    scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
    scrollArea->setGeometry(QRect(25, 25, 541, 361));
    scrollArea->setMinimumSize(QSize(200, 0));
    scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    scrollArea->setWidgetResizable(true);
    scrollArea->setAlignment(Qt::AlignCenter);
    scrollAreaWidgetContents = new QWidget();
    scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
    scrollAreaWidgetContents->setGeometry(QRect(0, 0, 539, 359));
    verticalLayout_2 = new QVBoxLayout(scrollAreaWidgetContents);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(11, 11, 11, 11);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    LabPicture = new QLabel(scrollAreaWidgetContents);
    LabPicture->setObjectName(QString::fromUtf8("LabPicture"));
    LabPicture->setScaledContents(false);
    LabPicture->setAlignment(Qt::AlignCenter);
    verticalLayout_2->addWidget(LabPicture);
    scrollArea->setWidget(scrollAreaWidgetContents);
    this->setCentralWidget(centralWidget);

    menuBar = new QMenuBar(this);
    menuBar->setObjectName(QString::fromUtf8("menuBar"));
    menuBar->setGeometry(QRect(0, 0, 934, 23));
    menuPic = new QMenu(menuBar);
    menuPic->setObjectName(QString::fromUtf8("menuPic"));
    menuView = new QMenu(menuBar);
    menuView->setObjectName(QString::fromUtf8("menuView"));
    this->setMenuBar(menuBar);
    mainToolBar = new QToolBar(this);
    mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
    mainToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    this->addToolBar(Qt::TopToolBarArea, mainToolBar);
    statusBar = new QStatusBar(this);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    this->setStatusBar(statusBar);
    dockWidget = new QDockWidget(this);
    dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
    dockWidget->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dockWidgetContents = new QWidget();
    dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
    verticalLayout = new QVBoxLayout(dockWidgetContents);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    verticalLayout->setContentsMargins(4, 2, 4, 2);
    treeFiles = new QTreeWidget(dockWidgetContents);

    QFont font1;
    font1.setBold(true);
    font1.setWeight(75);
    QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
    __qtreewidgetitem->setTextAlignment(1, Qt::AlignLeading | Qt::AlignVCenter);
    __qtreewidgetitem->setFont(1, font1);
    __qtreewidgetitem->setTextAlignment(0, Qt::AlignCenter);
    __qtreewidgetitem->setFont(0, font1);
    treeFiles->setHeaderItem(__qtreewidgetitem);
    QIcon icon11;
    icon11.addFile(QString::fromUtf8(":/images/icons/15.ico"), QSize(), QIcon::Normal, QIcon::Off);
    QIcon icon12;
    icon12.addFile(QString::fromUtf8(":/images/icons/31.ico"), QSize(), QIcon::Normal, QIcon::Off);
    QTreeWidgetItem *__qtreewidgetitem1 = new QTreeWidgetItem(treeFiles);
    __qtreewidgetitem1->setIcon(0, icon11);
    QTreeWidgetItem *__qtreewidgetitem2 = new QTreeWidgetItem(__qtreewidgetitem1);
    __qtreewidgetitem2->setIcon(0, icon);
    QTreeWidgetItem *__qtreewidgetitem3 = new QTreeWidgetItem(__qtreewidgetitem2);
    __qtreewidgetitem3->setIcon(0, icon12);
    QTreeWidgetItem *__qtreewidgetitem4 = new QTreeWidgetItem(__qtreewidgetitem1);
    QTreeWidgetItem *__qtreewidgetitem5 = new QTreeWidgetItem(__qtreewidgetitem4);
    __qtreewidgetitem5->setIcon(0, icon12);
    treeFiles->setObjectName(QString::fromUtf8("treeFiles"));
    treeFiles->setColumnCount(2);
    treeFiles->header()->setDefaultSectionSize(150);

    verticalLayout->addWidget(treeFiles);

    dockWidget->setWidget(dockWidgetContents);
    this->addDockWidget(Qt::LeftDockWidgetArea, dockWidget);

    menuBar->addAction(menuPic->menuAction());
    menuBar->addAction(menuView->menuAction());
    menuPic->addAction(actAddFolder);
    menuPic->addAction(actAddFiles);
    menuPic->addAction(actDeleteItem);
    menuPic->addAction(actScanItems);
    menuPic->addSeparator();
    menuPic->addAction(actQuit);
    menuView->addAction(actZoomIn);
    menuView->addAction(actZoomOut);
    menuView->addAction(actZoomRealSize);
    menuView->addAction(actZoomFitW);
    menuView->addAction(actZoomFitH);

    mainToolBar->addAction(actAddFolder);
    mainToolBar->addAction(actAddFiles);
    mainToolBar->addAction(actDeleteItem);
    mainToolBar->addAction(actScanItems);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actZoomIn);
    mainToolBar->addAction(actZoomOut);
    mainToolBar->addAction(actZoomRealSize);
    mainToolBar->addAction(actZoomFitW);
    mainToolBar->addAction(actZoomFitH);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actDockFloat);
    mainToolBar->addAction(actDockVisible);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actQuit);

    retranslateUi();
    QObject::connect(actQuit, SIGNAL(triggered()), this, SLOT(close()));

    QMetaObject::connectSlotsByName(this);
}

MainWindow::~MainWindow()
{
}

void MainWindow::retranslateUi()
{
    this->setWindowTitle(QCoreApplication::translate("MainWindow", "QTreeWidget\347\232\204\344\275\277\347\224\250", nullptr));
    actAddFolder->setText(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\347\233\256\345\275\225...", nullptr));
#if QT_CONFIG(tooltip)
    actAddFolder->setToolTip(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\347\233\256\345\275\225", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actAddFolder->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+F", nullptr));
#endif // QT_CONFIG(shortcut)
    actAddFiles->setText(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\346\226\207\344\273\266...", nullptr));
#if QT_CONFIG(tooltip)
    actAddFiles->setToolTip(QCoreApplication::translate("MainWindow", "\346\267\273\345\212\240\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actAddFiles->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+N", nullptr));
#endif // QT_CONFIG(shortcut)
    actZoomIn->setText(QCoreApplication::translate("MainWindow", "\346\224\276\345\244\247", nullptr));
#if QT_CONFIG(tooltip)
    actZoomIn->setToolTip(QCoreApplication::translate("MainWindow", "\346\224\276\345\244\247\345\233\276\347\211\207", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actZoomIn->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+I", nullptr));
#endif // QT_CONFIG(shortcut)
    actZoomOut->setText(QCoreApplication::translate("MainWindow", "\347\274\251\345\260\217", nullptr));
#if QT_CONFIG(tooltip)
    actZoomOut->setToolTip(QCoreApplication::translate("MainWindow", "\347\274\251\345\260\217\345\233\276\347\211\207", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actZoomOut->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+O", nullptr));
#endif // QT_CONFIG(shortcut)
    actZoomRealSize->setText(QCoreApplication::translate("MainWindow", "\345\256\236\351\231\205\345\244\247\345\260\217", nullptr));
#if QT_CONFIG(tooltip)
    actZoomRealSize->setToolTip(QCoreApplication::translate("MainWindow", "\345\233\276\347\211\207\345\256\236\351\231\205\345\244\247\345\260\217\346\230\276\347\244\272", nullptr));
#endif // QT_CONFIG(tooltip)
    actZoomFitW->setText(QCoreApplication::translate("MainWindow", "\351\200\202\345\220\210\345\256\275\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
    actZoomFitW->setToolTip(QCoreApplication::translate("MainWindow", "\351\200\202\345\220\210\345\256\275\345\272\246\346\230\276\347\244\272", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
    actZoomFitW->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+W", nullptr));
#endif // QT_CONFIG(shortcut)
    actDeleteItem->setText(QCoreApplication::translate("MainWindow", "\345\210\240\351\231\244\350\212\202\347\202\271", nullptr));
#if QT_CONFIG(tooltip)
    actDeleteItem->setToolTip(QCoreApplication::translate("MainWindow", "\345\210\240\351\231\244\350\212\202\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
    actQuit->setText(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272", nullptr));
#if QT_CONFIG(tooltip)
    actQuit->setToolTip(QCoreApplication::translate("MainWindow", "\351\200\200\345\207\272\346\234\254\347\263\273\347\273\237", nullptr));
#endif // QT_CONFIG(tooltip)
    actZoomFitH->setText(QCoreApplication::translate("MainWindow", "\351\200\202\345\220\210\351\253\230\345\272\246", nullptr));
#if QT_CONFIG(shortcut)
    actZoomFitH->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+H", nullptr));
#endif // QT_CONFIG(shortcut)
    actScanItems->setText(QCoreApplication::translate("MainWindow", "\351\201\215\345\216\206\350\212\202\347\202\271", nullptr));
#if QT_CONFIG(tooltip)
    actScanItems->setToolTip(QCoreApplication::translate("MainWindow", "\351\201\215\345\216\206\350\212\202\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
    actDockVisible->setText(QCoreApplication::translate("MainWindow", "\347\252\227\344\275\223\345\217\257\350\247\201", nullptr));
#if QT_CONFIG(tooltip)
    actDockVisible->setToolTip(QCoreApplication::translate("MainWindow", "\347\252\227\344\275\223\345\217\257\350\247\201", nullptr));
#endif // QT_CONFIG(tooltip)
    actDockFloat->setText(QCoreApplication::translate("MainWindow", "\347\252\227\344\275\223\346\265\256\345\212\250", nullptr));
#if QT_CONFIG(tooltip)
    actDockFloat->setToolTip(QCoreApplication::translate("MainWindow", "\347\252\227\344\275\223\346\265\256\345\212\250", nullptr));
#endif // QT_CONFIG(tooltip)
    LabPicture->setText(QString());
    menuPic->setTitle(QCoreApplication::translate("MainWindow", "\347\233\256\345\275\225\346\240\221", nullptr));
    menuView->setTitle(QCoreApplication::translate("MainWindow", "\350\247\206\345\233\276", nullptr));
    dockWidget->setWindowTitle(QCoreApplication::translate("MainWindow", "\345\233\276\347\211\207\347\233\256\345\275\225\346\240\221", nullptr));
    QTreeWidgetItem *___qtreewidgetitem = treeFiles->headerItem();
    ___qtreewidgetitem->setText(1, QCoreApplication::translate("MainWindow", "\350\212\202\347\202\271\347\261\273\345\236\213", nullptr));
    ___qtreewidgetitem->setText(0, QCoreApplication::translate("MainWindow", "\350\212\202\347\202\271", nullptr));

    const bool __sortingEnabled = treeFiles->isSortingEnabled();
    treeFiles->setSortingEnabled(false);
    QTreeWidgetItem *___qtreewidgetitem1 = treeFiles->topLevelItem(0);
    ___qtreewidgetitem1->setText(0, QCoreApplication::translate("MainWindow", "\345\233\276\347\211\207\346\226\207\344\273\266", nullptr));
    QTreeWidgetItem *___qtreewidgetitem2 = ___qtreewidgetitem1->child(0);
    ___qtreewidgetitem2->setText(0, QCoreApplication::translate("MainWindow", "\345\210\206\347\273\204\350\212\202\347\202\271", nullptr));
    QTreeWidgetItem *___qtreewidgetitem3 = ___qtreewidgetitem2->child(0);
    ___qtreewidgetitem3->setText(0, QCoreApplication::translate("MainWindow", "\345\233\276\347\211\207\350\212\202\347\202\271", nullptr));
    QTreeWidgetItem *___qtreewidgetitem4 = ___qtreewidgetitem1->child(1);
    ___qtreewidgetitem4->setText(0, QCoreApplication::translate("MainWindow", "\345\210\206\347\273\2042", nullptr));
    QTreeWidgetItem *___qtreewidgetitem5 = ___qtreewidgetitem4->child(0);
    ___qtreewidgetitem5->setText(0, QCoreApplication::translate("MainWindow", "\345\233\276\347\211\2072", nullptr));
    treeFiles->setSortingEnabled(__sortingEnabled);

} // retranslateUi

void MainWindow::iniTree()
{
    QString dataStr = ""; // Item的Data 存储的string
    treeFiles->clear();   // 清除目录树所有节点
    QIcon icon;
    icon.addFile(":/images/icons/15.ico"); // 设置ICON的图标
    QTreeWidgetItem *item = new QTreeWidgetItem(MainWindow::itTopItem);
    item->setIcon(MainWindow::colItem, icon);                 // 设置第1列的图标
    item->setText(MainWindow::colItem, "图片文件");           // 设置第1列的文字
    item->setText(MainWindow::colItemType, "type=itTopItem"); // 设置第2列的文字
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsAutoTristate);
    item->setCheckState(colItem, Qt::Checked); // 设置为选中

    item->setData(MainWindow::colItem, Qt::UserRole, QVariant(dataStr)); // 设置节点第1列的Qt::UserRole的Data
    treeFiles->addTopLevelItem(item);                                    // 添加顶层节点
}

void MainWindow::addFolderItem(QTreeWidgetItem *parItem, QString dirName)
{ // 添加一个目录节点
    QIcon icon(":/images/icons/open3.bmp");
    QString NodeText = getFinalFolderName(dirName); // 从一个完整目录名称里，获得最后的文件夹名称

    QTreeWidgetItem *item;                                                                                       // 节点
    item = new QTreeWidgetItem(MainWindow::itGroupItem);                                                         // 新建节点, 设定type为 itGroupItem
    item->setIcon(colItem, icon);                                                                                // 设置图标
    item->setText(colItem, NodeText);                                                                            // 最后的文件夹名称，第1列
    item->setText(colItemType, "type=itGroupItem");                                                              // 完整目录名称，第2列
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsAutoTristate); // 设置节点选项
    item->setCheckState(colItem, Qt::Checked);                                                                   // 节点选中
    item->setData(colItem, Qt::UserRole, QVariant(dirName));                                                     // 设置角色为Qt::UserRole的Data,存储完整目录名称

    parItem->addChild(item); // 在父节点下面添加子节点
}

QString MainWindow::getFinalFolderName(const QString &fullPathName)
{                                                  // 从一个完整目录名称里，获得最后的文件夹名称
    int cnt = fullPathName.length();               // 字符串长度
    int i = fullPathName.lastIndexOf("/");         //  最后一次出现的位置
    QString str = fullPathName.right(cnt - i - 1); // 获得最后的文件夹的名称
    return str;
}

void MainWindow::addImageItem(QTreeWidgetItem *parItem, QString aFilename)
{                                                     // 添加一个图片文件节点
    QIcon icon(":/images/icons/31.ico");              // ICON的图标
    QString NodeText = getFinalFolderName(aFilename); // 获得最后的文件名称

    QTreeWidgetItem *item;                                                                                       // 节点
    item = new QTreeWidgetItem(MainWindow::itImageItem);                                                         // 新建节点时设定类型为 itImageItem
    item->setIcon(colItem, icon);                                                                                // 设置图标
    item->setText(colItem, NodeText);                                                                            // 最后的文件夹名称
    item->setText(colItemType, "type=itImageItem");                                                              // 完整目录名称
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsAutoTristate); // 设置节点选项
    item->setCheckState(colItem, Qt::Checked);                                                                   // 节点选中

    item->setData(colItem, Qt::UserRole, QVariant(aFilename)); // 设置节点Qt::UserRole的Data,存储完整文件名称

    parItem->addChild(item); // 在父节点下面添加子节点
}

void MainWindow::displayImage(QTreeWidgetItem *item)
{                                                                    // 显示图片,节点item存储的图片文件名
    QString filename = item->data(colItem, Qt::UserRole).toString(); // 获取节点data里存的文件名
    LabFileName->setText(filename);
    curPixmap.load(filename); // 从文件载入图片
    // on_actZoomFitH_triggered(); // 自动适应高度显示

    actZoomFitH->setEnabled(true);
    actZoomFitW->setEnabled(true);
    actZoomIn->setEnabled(true);
    actZoomOut->setEnabled(true);
    actZoomRealSize->setEnabled(true);
}

void MainWindow::changeItemCaption(QTreeWidgetItem *item)
{                                            // 改变节点的标题文字
    QString str = "*" + item->text(colItem); // 节点标题前加“*”
    item->setText(colItem, str);             // 设置节点标题

    if (item->childCount() > 0)                      // 如果有子节点
        for (int i = 0; i < item->childCount(); i++) // 遍历子节点
            changeItemCaption(item->child(i));       // 调用自己，可重入的函数
}

void MainWindow::on_treeFiles_currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{ // 当前节点选择变化时触发
    Q_UNUSED(previous);
    if (current == NULL)
        return;

    int var = current->type(); // 节点的类型

    switch (var)
    {
    case itTopItem: // 顶层节点
        actAddFolder->setEnabled(true);
        actAddFiles->setEnabled(true);
        actDeleteItem->setEnabled(false); // 顶层节点不能删除
        break;

    case itGroupItem: // 文件组节点
        actAddFolder->setEnabled(true);
        actAddFiles->setEnabled(true);
        actDeleteItem->setEnabled(true);
        break;

    case itImageItem:                    // 图片文件节点
        actAddFolder->setEnabled(false); // 图片节点下不能添加目录节点
        actAddFiles->setEnabled(true);
        actDeleteItem->setEnabled(true);
        displayImage(current); // 显示图片
        break;
    }
}

void MainWindow::on_actAddFolder_triggered()
{                                                      // 选择一个文件夹,作为当前节点的子节点加入
    QString dir = QFileDialog::getExistingDirectory(); // 选择目录
    if (!dir.isEmpty())                                // 选择目录名称不为空
    {
        QTreeWidgetItem *parItem = treeFiles->currentItem(); // 当前节点
        addFolderItem(parItem, dir);                         // 在父节点下面添加一个组节点
    }
}

void MainWindow::on_actAddFiles_triggered()
{                                                                                                       // 添加图片文件节点
    QStringList files = QFileDialog::getOpenFileNames(this, "选择一个或多个文件", "", "Images(*.jpg)"); // 多选文件
    if (files.isEmpty())                                                                                // 如果一个文件都没选
        return;

    QTreeWidgetItem *parItem, *item; // 节点
    item = treeFiles->currentItem(); // 当前节点

    if (item->type() == itImageItem) // 若当前节点是图片节点，取其父节点作为父节点
        parItem = item->parent();
    else // 否则取当前节点为父节点
        parItem = item;

    for (int i = 0; i < files.size(); ++i)
    {
        QString aFilename = files.at(i);  // 得到StringList里的一行，也就是一个文件名
        addImageItem(parItem, aFilename); // 添加一个图片节点
    }
}

void MainWindow::on_actZoomOut_triggered()
{                              // 缩小显示
    pixRatio = pixRatio * 0.8; // 在当前比例基础上乘以0.8

    int w = pixRatio * curPixmap.width();  // 显示宽度
    int h = pixRatio * curPixmap.height(); // 显示高度

    QPixmap pix = curPixmap.scaled(w, h); // 图片缩放到指定高度和宽度，保持长宽比例

    LabPicture->setPixmap(pix);
}

void MainWindow::on_actZoomIn_triggered()
{                              // 放大显示
    pixRatio = pixRatio * 1.2; // 在当前比例基础上乘以0.8

    int w = pixRatio * curPixmap.width();  // 显示宽度
    int h = pixRatio * curPixmap.height(); // 显示高度

    QPixmap pix = curPixmap.scaled(w, h); // 图片缩放到指定高度和宽度，保持长宽比例
    LabPicture->setPixmap(pix);
}

void MainWindow::on_actZoomFitW_triggered()
{                                     // 适应宽度显示
    int w = scrollArea->width() - 20; // 得到scrollArea的高度
    int realw = curPixmap.width();    // 原始图片的实际宽度
    pixRatio = float(w) / realw;      // 当前显示比例，必须转换为浮点数

    QPixmap pix = curPixmap.scaledToWidth(w - 30);
    LabPicture->setPixmap(pix);
}

void MainWindow::on_actZoomFitH_triggered()
{                                   // 适应高度显示图片
    int H = scrollArea->height();   // 得到scrollArea的高度
    int realH = curPixmap.height(); // 原始图片的实际高度
    pixRatio = float(H) / realH;    // 当前显示比例，必须转换为浮点数

    QPixmap pix = curPixmap.scaledToHeight(H - 30); // 图片缩放到指定高度
    LabPicture->setPixmap(pix);                     // 设置Label的PixMap
}

void MainWindow::on_actZoomRealSize_triggered()
{                 // 实际大小显示
    pixRatio = 1; // 恢复显示比例为1
    LabPicture->setPixmap(curPixmap);
}

void MainWindow::on_actDeleteItem_triggered()
{                                                     // 删除节点
    QTreeWidgetItem *item = treeFiles->currentItem(); // 当前节点
    QTreeWidgetItem *parItem = item->parent();        // 父节点
    parItem->removeChild(item);                       // The removed item will not be deleted
    delete item;
}

void MainWindow::on_actScanItems_triggered()
{ // 遍历节点
    for (int i = 0; i < treeFiles->topLevelItemCount(); i++)
    {
        QTreeWidgetItem *item = treeFiles->topLevelItem(i); // 顶层节点
        changeItemCaption(item);                            // 更改节点标题
    }
}

void MainWindow::on_actDockVisible_toggled(bool arg1)
{ // 停靠区的可见性
    dockWidget->setVisible(arg1);
}

void MainWindow::on_dockWidget_visibilityChanged(bool visible)
{ // 停靠区可见性变化
    actDockVisible->setChecked(visible);
}

void MainWindow::on_dockWidget_topLevelChanged(bool topLevel)
{ // 停靠区浮动性变化
    actDockFloat->setChecked(topLevel);
}

void MainWindow::on_actDockFloat_triggered(bool checked)
{ // 停靠区浮动性
    dockWidget->setFloating(checked);
}
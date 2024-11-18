#include "mainwindow.h"

#include <QAction>
#include <QLineEdit>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QComboBox>
#include "mlabel.h"

void MainWindow::openTable()
{
    groupBox->setVisible(true);
    // 打开数据表
    tabModel = new QSqlTableModel(this, DB);                              // 数据表
    tabModel->setTable("employee");                                       // 设置数据表
    tabModel->setEditStrategy(QSqlTableModel::OnManualSubmit);            // 数据保存方式，OnManualSubmit , OnRowChange
    tabModel->setSort(tabModel->fieldIndex("empNo"), Qt::AscendingOrder); // 排序
    if (!(tabModel->select()))                                            // 查询数据
    {
        QMessageBox::critical(this, "错误信息",
                              "打开数据表错误,错误信息\n" + tabModel->lastError().text(),
                              QMessageBox::Ok, QMessageBox::NoButton);
        return;
    }

    // 字段显示名
    tabModel->setHeaderData(tabModel->fieldIndex("empNo"), Qt::Horizontal, "工号");
    tabModel->setHeaderData(tabModel->fieldIndex("Name"), Qt::Horizontal, "姓名");
    tabModel->setHeaderData(tabModel->fieldIndex("Gender"), Qt::Horizontal, "性别");
    tabModel->setHeaderData(tabModel->fieldIndex("Height"), Qt::Horizontal, "身高");
    tabModel->setHeaderData(tabModel->fieldIndex("Birthday"), Qt::Horizontal, "出生日期");
    tabModel->setHeaderData(tabModel->fieldIndex("Mobile"), Qt::Horizontal, "手机");
    tabModel->setHeaderData(tabModel->fieldIndex("Province"), Qt::Horizontal, "省份");
    tabModel->setHeaderData(tabModel->fieldIndex("City"), Qt::Horizontal, "城市");
    tabModel->setHeaderData(tabModel->fieldIndex("Department"), Qt::Horizontal, "部门");
    tabModel->setHeaderData(tabModel->fieldIndex("Education"), Qt::Horizontal, "学历");
    tabModel->setHeaderData(tabModel->fieldIndex("Salary"), Qt::Horizontal, "工资");

    tabModel->setHeaderData(tabModel->fieldIndex("Memo"), Qt::Horizontal, "备注"); // 这两个字段不再tableView中显示
    tabModel->setHeaderData(tabModel->fieldIndex("Photo"), Qt::Horizontal, "照片");

    theSelection = new QItemSelectionModel(tabModel); // 关联选择模型
    // theSelection当前项变化时触发currentChanged信号
    connect(theSelection, SIGNAL(currentChanged(QModelIndex, QModelIndex)),
            this, SLOT(on_currentChanged(QModelIndex, QModelIndex)));
    // 选择行变化时
    connect(theSelection, SIGNAL(currentRowChanged(QModelIndex, QModelIndex)),
            this, SLOT(on_currentRowChanged(QModelIndex, QModelIndex)));

    tableView->setModel(tabModel);                                   // 设置数据模型
    tableView->setSelectionModel(theSelection);                      // 设置选择模型
    tableView->setColumnHidden(tabModel->fieldIndex("Memo"), true);  // 隐藏列
    tableView->setColumnHidden(tabModel->fieldIndex("Photo"), true); // 隐藏列

    // tableView上为“性别”和“部门”两个字段设置自定义代理组件
    QStringList strList;
    strList << "男"
            << "女";
    bool isEditable = false;
    delegateSex.setItems(strList, isEditable);
    tableView->setItemDelegateForColumn(
        tabModel->fieldIndex("Gender"), &delegateSex); // Combbox选择型

    strList.clear();
    strList << "销售部"
            << "技术部"
            << "生产部"
            << "行政部";
    isEditable = true;
    delegateDepart.setItems(strList, isEditable);
    tableView->setItemDelegateForColumn(tabModel->fieldIndex("Department"), &delegateDepart); // Combbox选择型

    // 创建界面组件与数据模型的字段之间的数据映射
    dataMapper = new QDataWidgetMapper();
    dataMapper->setModel(tabModel);                             // 设置数据模型
    dataMapper->setSubmitPolicy(QDataWidgetMapper::AutoSubmit); //

    //    dataMapper->setItemDelegate(new QSqlRelationalDelegate(this)); //含有外键的
    // 界面组件与tabModel的具体字段之间的联系
    dataMapper->addMapping(dbSpinEmpNo, tabModel->fieldIndex("empNo"));
    dataMapper->addMapping(dbEditName, tabModel->fieldIndex("Name"));
    dataMapper->addMapping(dbComboSex, tabModel->fieldIndex("Gender"));

    dataMapper->addMapping(dbSpinHeight, tabModel->fieldIndex("Height"));
    dataMapper->addMapping(dbEditBirth, tabModel->fieldIndex("Birthday"));
    dataMapper->addMapping(dbEditMobile, tabModel->fieldIndex("Mobile"));

    dataMapper->addMapping(dbComboProvince, tabModel->fieldIndex("Province"));
    dataMapper->addMapping(dbEditCity, tabModel->fieldIndex("City"));
    dataMapper->addMapping(dbComboDep, tabModel->fieldIndex("Department"));

    dataMapper->addMapping(dbComboEdu, tabModel->fieldIndex("Education"));
    dataMapper->addMapping(dbSpinSalary, tabModel->fieldIndex("Salary"));

    dataMapper->addMapping(dbEditMemo, tabModel->fieldIndex("Memo"));

    //    dataMapper->addMapping(dbPhoto,tabModel->fieldIndex("Photo")); //图片无法直接映射

    dataMapper->toFirst(); // 移动到首记录

    getFieldNames(); // 获取字段名称列表，填充groupBoxSort组件

    // 更新actions和界面组件的使能状态
    actOpenDB->setEnabled(false);

    actOutputDB->setEnabled(true);
    actRecAppend->setEnabled(true);
    actRecInsert->setEnabled(true);
    actRecDelete->setEnabled(true);
    actScan->setEnabled(true);

    groupBoxSort->setEnabled(true);
    groupBoxFilter->setEnabled(true);
}

void MainWindow::getFieldNames()
{
    // 获取所有字段名称
    QSqlRecord emptyRec = tabModel->record(); // 获取空记录，只有字段名
    for (int i = 0; i < emptyRec.count(); i++)
    {
        comboFields->addItem(emptyRec.fieldName(i));
        comborefs->addItem(emptyRec.fieldName(i));
    }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    IintUI();
    setCentralWidget(splitter);
    //   tableView显示属性设置
    tableView->setSelectionBehavior(QAbstractItemView::SelectItems);
    tableView->setSelectionMode(QAbstractItemView::SingleSelection);
    tableView->setAlternatingRowColors(true);
    initConnects();
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_currentChanged(const QModelIndex &current, const QModelIndex &previous)
{ // 更新actPost和actCancel 的状态
    Q_UNUSED(current);
    Q_UNUSED(previous);
    // isDirty()函数通常用于检查一个窗口或者部件是否被改变
    actSubmit->setEnabled(tabModel->isDirty()); // 有未保存修改时可用
    actRevert->setEnabled(tabModel->isDirty());
}

void MainWindow::on_currentRowChanged(const QModelIndex &current, const QModelIndex &previous)
{
    Q_UNUSED(previous);
    // 行切换时的状态控制
    actRecDelete->setEnabled(current.isValid());
    actPhoto->setEnabled(current.isValid());
    actPhotoClear->setEnabled(current.isValid());

    if (!current.isValid())
    {
        dbLabPhoto->clear(); // 清除图片显示
        return;
    }
    // 数据变化
    dataMapper->setCurrentIndex(current.row()); // 更细数据映射的行号

    int curRecNo = current.row();                   // 获取行号
    QSqlRecord curRec = tabModel->record(curRecNo); // 获取当前记录

    if (curRec.isNull("Photo")) // 图片字段内容为空
        dbLabPhoto->clear();
    else
    {
        QByteArray data = curRec.value("Photo").toByteArray();
        QPixmap pic;
        pic.loadFromData(data);
        dbLabPhoto->setPixmap(pic.scaledToWidth(dbLabPhoto->size().width()));
    }
}

void MainWindow::on_actOpenDB_triggered()
{ // 打开数据表
    QString aFile = QFileDialog::getOpenFileName(this, "选择数据库文件", "",
                                                 "SQL Lite数据库(*.db *.db3)");
    if (aFile.isEmpty()) // 选择SQL Lite数据库文件
        return;

    // 打开数据库
    DB = QSqlDatabase::addDatabase("QSQLITE"); // 添加 SQL LITE数据库驱动
    DB.setDatabaseName(aFile);                 // 设置数据库名称
    //    DB.setHostName();
    //    DB.setUserName();
    //    DB.setPassword();
    if (!DB.open()) // 打开数据库
    {
        QMessageBox::warning(this, "错误", "打开数据库失败",
                             QMessageBox::Ok, QMessageBox::NoButton);
        return;
    }

    // 打开数据表
    openTable();
}

void MainWindow::on_actRecAppend_triggered()
{                                                             // 添加记录
    tabModel->insertRow(tabModel->rowCount(), QModelIndex()); // 在末尾添加一个记录

    QModelIndex curIndex = tabModel->index(tabModel->rowCount() - 1, 1);  // 创建最后一行的ModelIndex
    theSelection->clearSelection();                                       // 清空选择项
    theSelection->setCurrentIndex(curIndex, QItemSelectionModel::Select); // 设置刚插入的行为当前选择行

    int currow = curIndex.row();                                                // 获得当前行
    tabModel->setData(tabModel->index(currow, 0), 2000 + tabModel->rowCount()); // 自动生成编号
    tabModel->setData(tabModel->index(currow, 2), "男");
    // 插入行时设置缺省值，需要在primeInsert()信号里去处理
}

void MainWindow::on_actRecInsert_triggered()
{ // 插入记录
    QModelIndex curIndex = tableView->currentIndex();

    tabModel->insertRow(curIndex.row(), QModelIndex());

    theSelection->clearSelection(); // 清除已有选择
    theSelection->setCurrentIndex(curIndex, QItemSelectionModel::Select);
}

void MainWindow::on_actRevert_triggered()
{ // 取消修改
    tabModel->revertAll();
    actSubmit->setEnabled(false);
    actRevert->setEnabled(false);
}

void MainWindow::on_actSubmit_triggered()
{ // 保存修改
    bool res = tabModel->submitAll();

    if (!res)
        QMessageBox::information(this, "消息", "数据保存错误,错误信息\n" + tabModel->lastError().text(),
                                 QMessageBox::Ok, QMessageBox::NoButton);
    else
    {
        actSubmit->setEnabled(false);
        actRevert->setEnabled(false);
    }
}

void MainWindow::on_actRecDelete_triggered()
{                                                        // 删除当前记录
    QModelIndex curIndex = theSelection->currentIndex(); // 获取当前选择单元格的模型索引
    tabModel->removeRow(curIndex.row());                 // 删除最后一行
}

void MainWindow::on_actPhoto_triggered()
{
    // 设置照片
    QString aFile = QFileDialog::getOpenFileName(this, "选择图片文件", "", "照片(*.jpg *.png)");
    if (aFile.isEmpty())
        return;

    QByteArray data;
    QFile *file = new QFile(aFile); // fileName为二进制数据文件名
    file->open(QIODevice::ReadOnly);
    data = file->readAll();
    file->close();

    int curRecNo = theSelection->currentIndex().row();
    QSqlRecord curRec = tabModel->record(curRecNo); // 获取当前记录
    curRec.setValue("Photo", data);                 // 设置字段数据
    tabModel->setRecord(curRecNo, curRec);

    QPixmap pic;
    pic.load(aFile); // 在界面上显示
    dbLabPhoto->setPixmap(pic.scaledToWidth(dbLabPhoto->width()));
}

void MainWindow::on_actPhotoClear_triggered()
{
    int curRecNo = theSelection->currentIndex().row();
    QSqlRecord curRec = tabModel->record(curRecNo); // 获取当前记录

    curRec.setNull("Photo"); // 设置为空值
    tabModel->setRecord(curRecNo, curRec);

    dbLabPhoto->setPixmap(QPixmap());
}

void MainWindow::on_radioBtnAscend_clicked()
{ // 升序
    tabModel->setSort(comboFields->currentIndex(), Qt::AscendingOrder);
    tabModel->select();
}

void MainWindow::on_radioBtnDescend_clicked()
{ // 降序
    tabModel->setSort(comboFields->currentIndex(), Qt::DescendingOrder);
    tabModel->select();
}

void MainWindow::on_radioBtnMan_clicked()
{
    tabModel->setFilter(" Gender='男' ");
}

void MainWindow::on_radioBtnWoman_clicked()
{
    tabModel->setFilter(" Gender='女' ");
}

void MainWindow::on_radioBtnBoth_clicked()
{
    tabModel->setFilter("");
}

void MainWindow::on_comboFields_currentIndexChanged(int index)
{ // 选择字段进行排序
    if (radioBtnAscend->isChecked())
        tabModel->setSort(index, Qt::AscendingOrder);
    else
        tabModel->setSort(index, Qt::DescendingOrder);

    tabModel->select();
}

void MainWindow::on_actScan_triggered()
{ // 涨工资，记录遍历
    if (tabModel->rowCount() == 0)
        return;

    for (int i = 0; i < tabModel->rowCount(); i++)
    {
        QSqlRecord aRec = tabModel->record(i); // 获取当前记录
        float salary = aRec.value("Salary").toFloat();
        salary = salary * 1.1;
        aRec.setValue("Salary", salary);
        tabModel->setRecord(i, aRec);
    }

    if (tabModel->submitAll())
        QMessageBox::information(this, "消息", "涨工资计算完毕",
                                 QMessageBox::Ok, QMessageBox::NoButton);
}

void MainWindow::on_actOutputDB_triggered()
{
    if (tabModel == nullptr)
        return;
    // 获取原始数据库的结构
    QSqlRecord record = tabModel->record();
    // 断开现有Qt管理的数据库连接,并连接到一个新的数据库文件
    QString connectionName = "NewConnection";
    QSqlDatabase newDB = QSqlDatabase::addDatabase("QSQLITE", connectionName);
    QString saveFilePath = QFileDialog::getSaveFileName(nullptr,
                                                        "Save File",
                                                        QString(), // 默认保存的文件夹路径，这里使用默认值
                                                        "SQL Lite数据库(*.db *.db3)");
    newDB.setDatabaseName(saveFilePath);
    if (!newDB.open())
        return;

    QSqlQuery query(newDB);
    QString createTableQuery = QString("CREATE TABLE %1 (").arg(tabModel->tableName()); // 创造表
    for (int i = 0; i < record.count(); ++i)
    {
        QSqlField field = record.field(i);
        createTableQuery += QString("%1 %2").arg(field.name()).arg(field.typeID() == -1 ? "TEXT" : field.name());
        if (i != record.count() - 1)
        {
            createTableQuery += ", ";
        }
    }
    createTableQuery += ")"; // 获取每个字段的名称和数据类型，并添加到创建表的语句中。

    // 创建新表
    if (!query.exec(createTableQuery))
        return;

    // 开始事务 为了确保数据的一致性，代码开始一个事务。
    newDB.transaction();

    // 将数据导入新表
    for (int i = 0; i < tabModel->rowCount(); ++i)
    {
        record = tabModel->record(i);            // 获取第 i 行的数据记录
        QStringList valuePlaceHolders;           // 用于存储占位符的列表
        QList<QVariant> values;                  // 用于存储实际值的列表
        for (int j = 0; j < record.count(); ++j) // 遍历每个字段
        {
            valuePlaceHolders << "?";  // 添加占位符到列表中
            values << record.value(j); // 将字段的值添加到实际值列表中
        }
        // 遍历模型中的每一行数据，为每一行构造一个INSERT语句，并将其数据插入到新创建的表中
        QString insertSql = QString("INSERT INTO %1 VALUES (%2)").arg(tabModel->tableName()).arg(valuePlaceHolders.join(','));
        query.prepare(insertSql);               // 使用prepare和addBindValue来绑定实际的值到SQL语句中，以提高性能和安全性
        for (int k = 0; k < values.size(); ++k) // 遍历实际值列表
        {
            query.addBindValue(values.at(k)); // 将实际值绑定到SQL语句中
        }
        if (!query.exec()) // 如果执行失败
        {
            newDB.rollback(); // 回滚事务
            return;
        }
    }

    // 提交事务
    newDB.commit();

    // 关闭新数据库连接
    newDB.close();
    QSqlDatabase::removeDatabase(connectionName);
}

void MainWindow::on_initDB_triggered()
{
    // 断开现有Qt管理的数据库连接,并连接到一个新的数据库文件
    QString connectionName = "NewConnection";
    QSqlDatabase newDB = QSqlDatabase::addDatabase("QSQLITE", connectionName);
    QString saveFilePath = QFileDialog::getSaveFileName(nullptr,
                                                        "Save File",
                                                        QString(), // 默认保存的文件夹路径，这里使用默认值
                                                        "SQL Lite数据库(*.db *.db3)");
    newDB.setDatabaseName(saveFilePath);
    if (!newDB.open())
        return;

    QSqlQuery query(newDB);
    QString createTableQuery = QString("CREATE TABLE %1 (").arg("employee"); // 创造表
    QStringList lists = QStringList() << "empNo"
                                      << "Name"
                                      << "Gender"
                                      << "Height"
                                      << "Birthday"
                                      << "Mobile"
                                      << "Province"
                                      << "City"
                                      << "Department"
                                      << "Education"
                                      << "Salary"
                                      << "Photo"
                                      << "Memo";

    for (int i = 0; i < lists.count(); ++i)
    {
        createTableQuery += QString("%1 %2").arg(lists[i]).arg(lists[i]);
        if (i != lists.count() - 1)
        {
            createTableQuery += ", ";
        }
    }
    createTableQuery += ")"; // 获取每个字段的名称和数据类型，并添加到创建表的语句中。

    // 创建新表
    if (!query.exec(createTableQuery))
        return;
    // 关闭新数据库连接
    newDB.close();
    QSqlDatabase::removeDatabase(connectionName);
}

void MainWindow::on_reftext_changed(const QString &text)
{
    if (text.isEmpty())
        tabModel->setFilter("");
    else
    {
        QString filter = QString("%%%1%%").arg(text);
        tabModel->setFilter(comborefs->currentText() + " LIKE '" + filter + "'");
    }
}

void MainWindow::IintUI()
{
    this->resize(864, 503);
    QFont font;
    font.setPointSize(11);
    this->setFont(font);
    actOpenDB = new QAction(this);
    QIcon icon;
    icon.addFile(QString::fromUtf8(":/images/open"), QSize(), QIcon::Normal, QIcon::Off);
    actOpenDB->setIcon(icon);
    actOpenDB->setText("导入");
    actOutputDB = new QAction(this);
    QIcon icon0;
    icon0.addFile(QString::fromUtf8(":/images/open3"), QSize(), QIcon::Normal, QIcon::Off);
    actOutputDB->setIcon(icon0);
    actOutputDB->setText("导出");
    actOutputDB->setEnabled(false);
    actinitDB = new QAction(this);
    QIcon icon01;
    icon01.addFile(QString::fromUtf8(":/images/124"), QSize(), QIcon::Normal, QIcon::Off);
    actinitDB->setIcon(icon01);
    actinitDB->setText("初始化新库");
    actQuit = new QAction(this);
    QIcon icon1;
    icon1.addFile(QString::fromUtf8(":/images/exit"), QSize(), QIcon::Normal, QIcon::Off);
    actQuit->setIcon(icon1);
    actQuit->setText("退出");
    actRecAppend = new QAction(this);
    actRecAppend->setEnabled(false);
    QIcon icon2;
    icon2.addFile(QString::fromUtf8(":/images/add"), QSize(), QIcon::Normal, QIcon::Off);
    actRecAppend->setIcon(icon2);
    actRecAppend->setText("添加");
    actRecInsert = new QAction(this);
    actRecInsert->setEnabled(false);
    QIcon icon3;
    icon3.addFile(QString::fromUtf8(":/images/inster"), QSize(), QIcon::Normal, QIcon::Off);
    actRecInsert->setIcon(icon3);
    actRecInsert->setText("插入");
    actSubmit = new QAction(this);
    actSubmit->setEnabled(false);
    QIcon icon4;
    icon4.addFile(QString::fromUtf8(":/images/save1"), QSize(), QIcon::Normal, QIcon::Off);
    actSubmit->setIcon(icon4);
    actSubmit->setText("保存");
    actRevert = new QAction(this);
    actRevert->setEnabled(false);
    QIcon icon5;
    icon5.addFile(QString::fromUtf8(":/images/ubdo"), QSize(), QIcon::Normal, QIcon::Off);
    actRevert->setIcon(icon5);
    actRevert->setText("撤销");
    actRecDelete = new QAction(this);
    actRecDelete->setEnabled(false);
    QIcon icon6;
    icon6.addFile(QString::fromUtf8(":/images/delete"), QSize(), QIcon::Normal, QIcon::Off);
    actRecDelete->setIcon(icon6);
    actRecDelete->setText("删除");
    actPhoto = new QAction(this);
    actPhoto->setEnabled(false);
    QIcon icon7;
    icon7.addFile(QString::fromUtf8(":/images/00"), QSize(), QIcon::Normal, QIcon::Off);
    actPhoto->setIcon(icon7);
    actPhoto->setText("头像");
    actPhotoClear = new QAction(this);
    actPhotoClear->setEnabled(false);
    QIcon icon8;
    icon8.addFile(QString::fromUtf8(":/images/103"), QSize(), QIcon::Normal, QIcon::Off);
    actPhotoClear->setIcon(icon8);
    actPhotoClear->setText("删除图像");
    actScan = new QAction(this);
    actScan->setEnabled(false);
    QIcon icon9;
    icon9.addFile(QString::fromUtf8(":/images/up"), QSize(), QIcon::Normal, QIcon::Off);
    actScan->setIcon(icon9);
    actScan->setText("涨工资");
    centralWidget = new QWidget(this);
    splitter = new QSplitter(centralWidget);
    splitter->setGeometry(QRect(10, 10, 781, 381));
    splitter->setOrientation(Qt::Horizontal);
    layoutWidget = new QWidget(splitter);
    verticalLayout_2 = new QVBoxLayout(layoutWidget);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(11, 11, 11, 11);
    verticalLayout_2->setContentsMargins(0, 0, 0, 0);
    groupBox_2 = new QGroupBox(layoutWidget);
    horizontalLayout_2 = new QHBoxLayout(groupBox_2);
    horizontalLayout_2->setSpacing(6);
    horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_2->setContentsMargins(2, 2, -1, 2);
    groupBoxSort = new QGroupBox(groupBox_2);
    groupBoxSort->setTitle("排序");
    groupBoxSort->setEnabled(false);
    gridLayout_3 = new QGridLayout(groupBoxSort);
    gridLayout_3->setSpacing(6);
    gridLayout_3->setContentsMargins(11, 11, 11, 11);
    label_14 = new QLabel(groupBoxSort);
    label_14->setText("排序字段");
    gridLayout_3->addWidget(label_14, 0, 0, 1, 1);

    comboFields = new QComboBox(groupBoxSort);

    gridLayout_3->addWidget(comboFields, 0, 1, 1, 2);

    horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    gridLayout_3->addItem(horizontalSpacer, 1, 0, 1, 1);

    radioBtnAscend = new QRadioButton(groupBoxSort);
    radioBtnAscend->setChecked(true);
    radioBtnAscend->setText("升序");
    gridLayout_3->addWidget(radioBtnAscend, 1, 1, 1, 1);

    radioBtnDescend = new QRadioButton(groupBoxSort);
    radioBtnDescend->setText("降序");
    gridLayout_3->addWidget(radioBtnDescend, 1, 2, 1, 1);

    horizontalLayout_2->addWidget(groupBoxSort);

    groupBoxFilter = new QGroupBox(groupBox_2);
    groupBoxFilter->setEnabled(false);
    groupBoxFilter->setTitle("数据过滤");
    gridLayout_2 = new QGridLayout(groupBoxFilter);
    gridLayout_2->setSpacing(6);
    gridLayout_2->setContentsMargins(11, 11, 11, 11);
    radioBtnMan = new QRadioButton(groupBoxFilter);
    radioBtnMan->setText("男");

    gridLayout_2->addWidget(radioBtnMan, 0, 0, 1, 1);

    radioBtnWoman = new QRadioButton(groupBoxFilter);
    radioBtnWoman->setText("女");
    gridLayout_2->addWidget(radioBtnWoman, 0, 1, 1, 1);

    radioBtnBoth = new QRadioButton(groupBoxFilter);
    radioBtnBoth->setChecked(true);
    radioBtnBoth->setText("全显示");
    gridLayout_2->addWidget(radioBtnBoth, 0, 2, 1, 1);

    comborefs = new QComboBox(groupBoxFilter);
    gridLayout_2->addWidget(comborefs, 1, 0, 1, 1);
    reflineEdit = new QLineEdit(groupBoxFilter);
    reflineEdit->setPlaceholderText("过滤文本");
    gridLayout_2->addWidget(reflineEdit, 1, 1, 1, 1);

    // horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    // gridLayout_2->addItem(horizontalSpacer_2, 1, 1, 1, 1);

    horizontalLayout_2->addWidget(groupBoxFilter);

    horizontalSpacer_3 = new QSpacerItem(45, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_2->addItem(horizontalSpacer_3);

    verticalLayout_2->addWidget(groupBox_2);

    tableView = new QTableView(layoutWidget);
    tableView->setEditTriggers(QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed | QAbstractItemView::SelectedClicked);

    verticalLayout_2->addWidget(tableView);

    splitter->addWidget(layoutWidget);
    groupBox = new QGroupBox(splitter);
    groupBox->setVisible(false);
    horizontalLayout = new QHBoxLayout(groupBox);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    gridLayout = new QGridLayout();
    gridLayout->setSpacing(6);
    label = new QLabel(groupBox);
    label->setText("工 号");
    gridLayout->addWidget(label, 0, 0, 1, 1);

    dbSpinEmpNo = new QSpinBox(groupBox);
    dbSpinEmpNo->setMinimum(1);
    dbSpinEmpNo->setMaximum(10000);

    gridLayout->addWidget(dbSpinEmpNo, 0, 1, 1, 1);

    label_2 = new QLabel(groupBox);
    label_2->setText("姓  名");
    gridLayout->addWidget(label_2, 1, 0, 1, 1);

    dbEditName = new QLineEdit(groupBox);

    gridLayout->addWidget(dbEditName, 1, 1, 1, 1);

    label_3 = new QLabel(groupBox);
    label_3->setText("性  别");
    gridLayout->addWidget(label_3, 2, 0, 1, 1);

    dbComboSex = new QComboBox(groupBox);
    dbComboSex->addItem("男");
    dbComboSex->addItem("女");

    gridLayout->addWidget(dbComboSex, 2, 1, 1, 1);

    label_4 = new QLabel(groupBox);
    label_4->setText("身  高");
    gridLayout->addWidget(label_4, 3, 0, 1, 1);

    dbSpinHeight = new QDoubleSpinBox(groupBox);
    dbSpinHeight->setMaximum(3.000000000000000);
    dbSpinHeight->setSingleStep(0.010000000000000);
    dbSpinHeight->setValue(1.560000000000000);

    gridLayout->addWidget(dbSpinHeight, 3, 1, 1, 1);

    label_5 = new QLabel(groupBox);
    label_5->setText("出生日期");

    gridLayout->addWidget(label_5, 4, 0, 1, 1);

    dbEditBirth = new QDateEdit(groupBox);
    dbEditBirth->setCalendarPopup(true);
    dbEditBirth->setDate(QDate(2017, 2, 20));

    gridLayout->addWidget(dbEditBirth, 4, 1, 1, 1);

    label_10 = new QLabel(groupBox);
    label_10->setText("手机号");
    gridLayout->addWidget(label_10, 5, 0, 1, 1);

    dbEditMobile = new QLineEdit(groupBox);

    gridLayout->addWidget(dbEditMobile, 5, 1, 1, 1);

    label_7 = new QLabel(groupBox);
    label_7->setText("出生省份");
    gridLayout->addWidget(label_7, 6, 0, 1, 1);

    dbComboProvince = new QComboBox(groupBox);
    dbComboProvince->addItem("河北");
    dbComboProvince->addItem("河南");
    dbComboProvince->addItem("山东");
    dbComboProvince->addItem("湖南");
    dbComboProvince->addItem("湖北");
    dbComboProvince->addItem("广东");
    dbComboProvince->setEditable(true);

    gridLayout->addWidget(dbComboProvince, 6, 1, 1, 1);

    label_8 = new QLabel(groupBox);
    label_8->setText("城  市");
    gridLayout->addWidget(label_8, 7, 0, 1, 1);

    dbEditCity = new QLineEdit(groupBox);

    gridLayout->addWidget(dbEditCity, 7, 1, 1, 1);

    label_6 = new QLabel(groupBox);
    label_6->setText("部  门");
    gridLayout->addWidget(label_6, 8, 0, 1, 1);

    dbComboDep = new QComboBox(groupBox);
    QStringList strList = QStringList() << "销售部"
                                        << "技术部"
                                        << "生产部"
                                        << "行政部";
    dbComboDep->addItems(strList);
    dbComboDep->setEditable(true);

    gridLayout->addWidget(dbComboDep, 8, 1, 1, 1);

    dbEditMemo = new QPlainTextEdit(groupBox);
    dbEditMemo->setMaximumSize(QSize(16777215, 16777215));

    gridLayout->addWidget(dbEditMemo, 11, 1, 1, 1);

    label_9 = new QLabel(groupBox);
    label_9->setText("备 注");
    gridLayout->addWidget(label_9, 11, 0, 1, 1);

    label_11 = new QLabel(groupBox);
    label_11->setText("学  历");
    gridLayout->addWidget(label_11, 9, 0, 1, 1);

    dbSpinSalary = new QSpinBox(groupBox);
    dbSpinSalary->setMinimum(1000);
    dbSpinSalary->setMaximum(50000);
    dbSpinSalary->setSingleStep(100);
    dbSpinSalary->setValue(1000);

    gridLayout->addWidget(dbSpinSalary, 10, 1, 1, 1);

    label_12 = new QLabel(groupBox);
    label_12->setText("工 资");
    gridLayout->addWidget(label_12, 10, 0, 1, 1);

    dbComboEdu = new QComboBox(groupBox);
    dbComboEdu->addItem("小学");
    dbComboEdu->addItem("初中");
    dbComboEdu->addItem("高中");
    dbComboEdu->addItem("大专");
    dbComboEdu->addItem("本科");
    dbComboEdu->addItem("研究生");
    dbComboEdu->addItem("博士");
    dbComboEdu->setEditable(true);

    gridLayout->addWidget(dbComboEdu, 9, 1, 1, 1);

    horizontalLayout->addLayout(gridLayout);

    verticalLayout = new QVBoxLayout();
    verticalLayout->setSpacing(6);
    label_13 = new QLabel(groupBox);
    label_13->setText("照  片");
    verticalLayout->addWidget(label_13);

    dbLabPhoto = new MyLable(groupBox);
    dbLabPhoto->setMinimumSize(QSize(150, 50));
    dbLabPhoto->setMaximumSize(QSize(350, 16777215));

    verticalLayout->addWidget(dbLabPhoto);

    verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout->addItem(verticalSpacer);

    horizontalLayout->addLayout(verticalLayout);

    splitter->addWidget(groupBox);
    this->setCentralWidget(centralWidget);
    menuBar = new QMenuBar(this);
    menuBar->setGeometry(QRect(0, 0, 864, 23));
    this->setMenuBar(menuBar);
    statusBar = new QStatusBar(this);
    this->setStatusBar(statusBar);
    mainToolBar = new QToolBar(this);
    mainToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    this->addToolBar(Qt::TopToolBarArea, mainToolBar);
#if QT_CONFIG(shortcut)
    label->setBuddy(dbSpinEmpNo);
    label_2->setBuddy(dbEditName);
    label_3->setBuddy(dbComboSex);
    label_4->setBuddy(dbSpinHeight);
    label_5->setBuddy(dbEditBirth);
    label_10->setBuddy(dbEditMobile);
    label_7->setBuddy(dbComboProvince);
    label_8->setBuddy(dbEditCity);
    label_6->setBuddy(dbComboDep);
    label_9->setBuddy(dbEditMemo);
    label_11->setBuddy(dbComboEdu);
    label_12->setBuddy(dbSpinSalary);
#endif // QT_CONFIG(shortcut)

    mainToolBar->addAction(actOpenDB);
    mainToolBar->addAction(actOutputDB);
    mainToolBar->addAction(actinitDB);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actRecAppend);
    mainToolBar->addAction(actRecInsert);
    mainToolBar->addAction(actRecDelete);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actPhoto);
    mainToolBar->addAction(actPhotoClear);
    mainToolBar->addAction(actScan);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actSubmit);
    mainToolBar->addAction(actRevert);
    mainToolBar->addSeparator();
    mainToolBar->addAction(actQuit);
}

void MainWindow::initConnects()
{
    connect(actRecAppend, &QAction::triggered, this, &MainWindow::on_actRecAppend_triggered);
    connect(actRecInsert, &QAction::triggered, this, &MainWindow::on_actRecInsert_triggered);
    connect(actRevert, &QAction::triggered, this, &MainWindow::on_actRevert_triggered);
    connect(actSubmit, &QAction::triggered, this, &MainWindow::on_actSubmit_triggered);
    connect(actRecDelete, &QAction::triggered, this, &MainWindow::on_actRecDelete_triggered);
    connect(actPhoto, &QAction::triggered, this, &MainWindow::on_actPhoto_triggered);
    connect(actPhotoClear, &QAction::triggered, this, &MainWindow::on_actPhotoClear_triggered);
    connect(actScan, &QAction::triggered, this, &MainWindow::on_actScan_triggered);
    connect(actOpenDB, &QAction::triggered, this, &MainWindow::on_actOpenDB_triggered);
    connect(actOutputDB, &QAction::triggered, this, &MainWindow::on_actOutputDB_triggered);
    connect(actinitDB, &QAction::triggered, this, &MainWindow::on_initDB_triggered);
    connect(actQuit, &QAction::triggered, [&]()
            { close(); });

    connect(radioBtnAscend, &QRadioButton::clicked, this, &MainWindow::on_radioBtnAscend_clicked);
    connect(radioBtnDescend, &QRadioButton::clicked, this, &MainWindow::on_radioBtnDescend_clicked);
    connect(radioBtnMan, &QRadioButton::clicked, this, &MainWindow::on_radioBtnMan_clicked);
    connect(radioBtnWoman, &QRadioButton::clicked, this, &MainWindow::on_radioBtnWoman_clicked);
    connect(radioBtnBoth, &QRadioButton::clicked, this, &MainWindow::on_radioBtnBoth_clicked);

    connect(comboFields, SIGNAL(currentIndexChanged(int)), this, SLOT(on_comboFields_currentIndexChanged(int)));

    connect(dbLabPhoto, SIGNAL(changePixmp()), this, SLOT(on_actPhoto_triggered()));
    connect(dbLabPhoto, SIGNAL(delPixmp()), this, SLOT(on_actPhotoClear_triggered()));

    connect(reflineEdit, &QLineEdit::textChanged, this, &MainWindow::on_reftext_changed);
}

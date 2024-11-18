@[toc]
## 示例图

### 一文带你实战qt数据库，使用到了类有SQSqlTableModel，QDataWidgetMapper，QSqlDatabase，QItemSelectionModel等以及多个常见的窗口控件
- **主要实现功能点**
	1. **导入导出**与初始化数据库
	2. 数据的**增删改查**
	3. 数据的撤销与保存
	4. 数据的排序与过滤
	5. 图片的自定义行为
	6. 控件与数据之间的交互
- **文末我会提供源码给大家参考**
## 入门
- 简单的入门QT数据库我们从了解Q==SQLite==与==QSqlTableModel==开始。
### SQLite
- 一个轻量级的数据库引擎，它的数据库是一个单一文件，因此非常适合移动应用和嵌入式设备。Qt 通过 SQL 模块提供对 SQLite 的接口，可以执行**创建、查询、更新和删除**（==CRUD==）操作，以及管理数据库连接和事务。
以下是使用 Qt SQLite 数据库的基本步骤：
1. 包含必要的头文件：
```cpp
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
```
1. 创建数据库连接：
```cpp
QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
db.setDatabaseName("your_database.db");
if (!db.open()) {
    qDebug() << "Error: " << db.lastError();
}
```
1. 执行 SQL 语句创建表：
```cpp
QSqlQuery query;
if (!query.exec("CREATE TABLE IF NOT EXISTS people ("
                 "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                 "name TEXT NOT NULL, "
                 "age INTEGER NOT NULL)")) {
    qDebug() << "Error: " << query.lastError();
}
```
1. 插入数据：
```cpp
query.prepare("INSERT INTO people (name, age) VALUES (?, ?)");
query.addBindValue("Alice");
query.addBindValue(30);
if (!query.exec()) {
    qDebug() << "Error: " << query.lastError();
}
```
1. 查询数据：
```cpp
if (query.exec("SELECT id, name, age FROM people")) {
    while (query.next()) {
        int id = query.value(0).toInt();
        QString name = query.value(1).toString();
        int age = query.value(2).toInt();
        qDebug() << id << name << age;
    }
} else {
    qDebug() << "Error: " << query.lastError();
}
```
1. 更新数据：
```cpp
query.prepare("UPDATE people SET age = ? WHERE name = ?");
query.addBindValue(31);
query.addBindValue("Alice");
if (!query.exec()) {
    qDebug() << "Error: " << query.lastError();
}
```
1. 删除数据：
```cpp
query.prepare("DELETE FROM people WHERE name = ?");
query.addBindValue("Alice");
if (!query.exec()) {
    qDebug() << "Error: " << query.lastError();
}
```
1. 关闭数据库连接：
```cpp
db.close();
```
- **Qt 的数据库操作遵循了面向对象的原则，通过 `QSqlQuery` 对象来执行 SQL 语句，并且可以使用预处理语句来防止 SQL 注入攻击。在处理数据库时，应当注意错误处理，确保程序的健壮性。此外，Qt 的数据库操作是非阻塞的，因此在执行长时间操作时，可能需要考虑异步执行**。

### QSqlTableModel
- `QSqlTableModel` 是 Qt SQL 模块中的一个类，它为 SQL 数据库提供了一个可编辑的数据模型。这个模型与 `QTableView` 等视图部件一起使用，可以方便地实现数据库表格的显示和编辑。`QSqlTableModel` 继承自 `QSqlQueryModel`，并添加了编辑功能，包括插入、删除和修改行。
以下是使用 `QSqlTableModel` 的一些基本步骤：
1. 包含必要的头文件：
```cpp
#include <QSqlTableModel>
#include <QTableView>
```
2. 创建 QSqlTableModel 实例并指定数据库表：
```cpp
QSqlTableModel *model = new QSqlTableModel(this);
model->setTable("people");
model->setEditStrategy(QSqlTableModel::OnFieldChange); // 可选，设置编辑策略
model->select(); // 选择数据
```
3. 创建 QTableView 实例并设置模型：
```cpp
QTableView *view = new QTableView;
view->setModel(model);
view->show();
```
4. （可选）设置模型的数据校验器和委托：
```cpp
// 设置年龄列的校验器，限制年龄在0到150之间
QIntValidator *ageValidator = new QIntValidator(0, 150, this);
model->setValidator(ageValidator, model->fieldIndex("age"));
// 设置姓名列的委托，使用 QLineEdit 来编辑
QLineEdit *nameEdit = new QLineEdit;
view->setItemDelegate(new QStyledItemDelegate(nameEdit), model->fieldIndex("name"));
```
5. 提交和撤销更改：
```cpp
// 提交所有更改到数据库
if (model->submitAll()) {
    qDebug() << "Changes submitted successfully";
} else {
    qDebug() << "Failed to submit changes";
}
// 撤销所有未提交的更改
model->revertAll();
```
6. 处理模型的信号：
```cpp
// 连接模型的数据改变信号
connect(model, &QSqlTableModel::dataChanged,
        this, &YourClass::onDataChanged);
```
- `QSqlTableModel` 的 `setEditStrategy` 函数可以设置为不同的编辑策略，例如：
`QSqlTableModel::OnManualSubmit`：手动提交更改。
`QSqlTableModel::OnRowChange`：当行更改时自动提交。
`QSqlTableModel::OnFieldChange`：当字段更改时自动提交。
- **在使用 `QSqlTableModel` 时，确保你已经创建了数据库连接，并且数据库表已经存在**。`QSqlTableModel` 提供了一个方便的方式来处理数据库的 ==CRUD== 操作，并且可以与 Qt 的视图部件无缝集成，以实现数据的可视化编辑。
## 实战
- **了解完`QSqlTableModel`与`QSqlDatabase`类后基本上可以上手了，但前提是你必须知道数据库的语句与语义。`QSqlTableModel`基本上把数据库的常见操作都封装了，你只需要会调接口就行。使用`QDataWidgetMapper`做数据映射，可以很方便的操作表格和文本框的数据，但需要注意的是图片是无法映射的，要单独处理**
- 篇幅限制下面只贴出一些功能点
### 导入
```cpp
void MainWindow::on_actOpenDB_triggered()
{ // 打开数据表
    QString aFile = QFileDialog::getOpenFileName(this, "选择数据库文件", "",
                                                 "SQL Lite数据库(*.db *.db3)");
    if (aFile.isEmpty()) // 选择SQL Lite数据库文件
        return;
    // 打开数据库
    DB = QSqlDatabase::addDatabase("QSQLITE"); // 添加 SQL LITE数据库驱动
    DB.setDatabaseName(aFile);                 // 设置数据库名称
    //    DB.setHostName();//若是需要用户密码
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
- `openTable()`是非常关键的函数，里面交代了`QSqlTableModel`的设置，代码过长，在源码中。
```
### 导出
```cpp
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
```
## 总结
- **知识理应共享**
- 上述示例[源码](https://gitee.com/shan-jie6/my-case/tree/master/QT/database)在此，使用的==cmake==在==vscode==运行的，功能点还能再加，也有很多优化的地方。
- 秉承着学习的态度，这是参考了一个[demo](https://gitee.com/shan-jie6/my-case/tree/master/demo/QT5.15Sample2023/chap11Database/samp11_1Table)实现的，使用==qrc==运行，在此基础上加了一些功能和一些优化。
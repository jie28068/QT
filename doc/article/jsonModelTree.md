[TOC]

### 效果图

### 概述

- 本案例在此开源项目[QJsonModel](https://github.com/dridk/QJsonModel)上实现，`QJsonModel`是一个基于`QAbstractItemModel`的`JSON`数据模型，它提供了一种简单的方式来将`JSON`数据可视化，功能简单来说就是读取`JSON`文件显示与修改。
- 根据具体的需求，修改了部分代码，添加了一些功能点。
    1. 添加了菜单，==可增删数据==
    2. 自定义约束数据类型
    3. 添加新旧值显示
    4. 自定义表头名称
    5. 鼠标悬浮提示
    6. ...

- 总的来说就是使用的==model-Viwe==架构，通过`QJsonModel`将`JSON`数据可视化，然后通过自定义的`QTreeView`来显示数据，，通过`QStyledItemDelegate`来添加自定义委托。

### 部分代码

- 右键菜单

```cpp
void CarTreeView::contextMenuEvent(QContextMenuEvent *event)
{
    /// 判断是否可编辑,不可编辑时，不响应右键菜单
    if (editTriggers() == QAbstractItemView::NoEditTriggers)
    {
        event->ignore();
        return;
    }
    // 获取点击的项
    QModelIndex index = indexAt(event->pos());
    QJsonModel *model = qobject_cast<QJsonModel *>(this->model());

    if (index.isValid() && model)
    {
        ////xxxx
    }
}

```

- 删除指定行

```cpp
  void QJsonModel::removeItem(const QModelIndex &index)
  {
    if (!index.isValid())
        return;

    QJsonTreeItem *item = static_cast<QJsonTreeItem *>(index.internalPointer());
    QJsonTreeItem *parentItem = item->parent();

    beginRemoveRows(index.parent(), item->row(), item->row());
    parentItem->mChilds.removeAt(item->row());
    delete item;
    endRemoveRows();
  }
``

- 为添加数组子项
```cpp
void QJsonModel::addArrayItem(const QModelIndex &index, QJsonObject jsonObject, const QString &key)
{
  if (!index.isValid())
    return;
  QJsonTreeItem *item = static_cast<QJsonTreeItem *>(index.internalPointer());

  // 确认当前item是数组类型
  if (!item && item->type() != QJsonValue::Array)
    return;
  // 创建新的QJsonTreeItem对象来表示QJsonObject
  QJsonTreeItem *newBowlItem = new QJsonTreeItem(item);
  newBowlItem->setType(QJsonValue::Object);
  int count = item->childCount();
  newBowlItem->setKey(QString::number(count));

  // 添加新的QJsonTreeItem对象到item的子项列表中
  QJsonTreeItem *newItem = new QJsonTreeItem(newBowlItem);
  newItem->setType(QJsonValue::Object);
  newItem->setKey(key);

  newBowlItem->appendChild(newItem);

  // 将QJsonObject的每个键值对添加到新的QJsonTreeItem中
  for (auto it = jsonObject.begin(); it != jsonObject.end(); ++it)
  {
    QJsonTreeItem *childItem = new QJsonTreeItem(newItem);
    childItem->setKey(it.key());
    childItem->setValue(it.value().toVariant());
    childItem->setType(it.value().type());
    newItem->appendChild(childItem);
  }

  // 将新的QJsonTreeItem添加到当前item的子项中
  item->appendChild(newBowlItem);

  // 通知模型数据已更改
  beginInsertRows(index, count, count);
  endInsertRows();
}
```

### 总结

- 知识理应共享，源码在此[点我](https://gitee.com/shan-jie6/my-case/tree/master/QT/carbonBlockHMI)。

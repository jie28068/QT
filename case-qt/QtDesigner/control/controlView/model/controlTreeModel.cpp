#include "controlTreeModel.h"
#include "controlTreeItem.h"

#include "globalDefinition.h"
FloatTreeModel::FloatTreeModel(const QStringList &headers, QObject *parent)
    : QAbstractItemModel(parent)
{
    _headers = headers;
    _rootItem = new FloatTreeItem();
}

FloatTreeModel::~FloatTreeModel()
{
    delete _rootItem;
}

FloatTreeItem *FloatTreeModel::itemFromIndex(const QModelIndex &index) const
{
    if (index.isValid())
    {
        FloatTreeItem *item = static_cast<FloatTreeItem *>(index.internalPointer());
        return item;
    }
    return _rootItem;
}

FloatTreeItem *FloatTreeModel::root()
{
    return _rootItem;
}

// 获取表头数据
QVariant FloatTreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal)
    {
        if (role == Qt::DisplayRole)
        {
            return _headers.at(section);
        }
    }
    return QVariant();
}

// 获取index.row行，index.column列数据
QVariant FloatTreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    FloatTreeItem *item = itemFromIndex(index);
    if (role == Qt::DisplayRole)
    {
        return item->data(index.column());
    }
    else if (role == Qt::CheckStateRole)
    {
        if (item->checkable(index.column()))
        {
            return item->isChecked() ? Qt::Checked : Qt::Unchecked;
        }
        return QVariant();
    }
    else if (role == Qt::DecorationRole)
    {
        return item->getIcon(index.column());
    }
    return QVariant();
}

// 在parent节点下，第row行，第column列位置上创建索引
QModelIndex FloatTreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    FloatTreeItem *parentItem = itemFromIndex(parent);
    FloatTreeItem *item = parentItem->child(row);
    if (item)
        return createIndex(row, column, item);
    else
        return QModelIndex();
}

// 创建index的父索引
QModelIndex FloatTreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    FloatTreeItem *item = itemFromIndex(index);
    FloatTreeItem *parentItem = item->parent();

    if (parentItem == _rootItem)
        return QModelIndex();
    return createIndex(parentItem->row(), 0, parentItem);
}

// 获取索引parent下有多少行
int FloatTreeModel::rowCount(const QModelIndex &parent) const
{
    if (parent.column() > 0)
        return 0;

    FloatTreeItem *item = itemFromIndex(parent);
    return item->childCount();
}

// 返回索引parent下有多少列
int FloatTreeModel::columnCount(const QModelIndex &parent) const
{
    return _headers.size();
}

bool FloatTreeModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (!index.isValid())
        return false;

    if (role == Qt::CheckStateRole)
    {
        FloatTreeItem *item = itemFromIndex(index);
        if (!item->checkable(index.column()))
        {
            return false;
        }

        item->setChecked(value.toInt() == Qt::Checked);
        emit dataChanged(index, index);
        return true;
    }
    return false;
}

Qt::ItemFlags FloatTreeModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return QAbstractItemModel::flags(index);

    Qt::ItemFlags flags = QAbstractItemModel::flags(index);
    FloatTreeItem *item = itemFromIndex(index);
    if (item->checkable(index.column()))
    {
        flags |= Qt::ItemIsUserCheckable;
    }
    return flags;
}

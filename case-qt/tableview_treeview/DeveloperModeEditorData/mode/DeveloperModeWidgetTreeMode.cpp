#include "DeveloperModeWidgetTreeMode.h"

#include "KLModelDefinitionCore.h"
#include "KLModelDefinitionCore/GlobalAssistant.h"
#include <QBrush>
#include <QIcon>
#include <QMimeData>

QString m_strFilterString;

using namespace Kcc::BlockDefinition;

TreeModel::TreeModel(QObject* parent) : QAbstractItemModel(parent)
{
    m_rootItem = new TreeItem("root", TreeItem::Unknown);
    m_header = QStringList() << "节点"
                             << "节点类型";
}

TreeModel::~TreeModel()
{
    delete m_rootItem;
}

TreeItem* TreeModel::itemFromIndex(const QModelIndex& index) const
{
    if (index.isValid()) {
        TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
        return item;
    } else {
        return m_rootItem;
    }
}

TreeItem* TreeModel::rootItem()
{
    return m_rootItem;
}

// 获取index.row行，index.column列数据
QVariant TreeModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    TreeItem* item = itemFromIndex(index);
    if (item) {
        switch (role) {
        case Qt::DisplayRole:
            return item->data(index.column());
        default:
            break;
        }
        return QVariant();
    }

    return QVariant();
}

// 在parent节点下，第row行，第column列位置上创建索引
QModelIndex TreeModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    TreeItem* parentItem = itemFromIndex(parent);
    TreeItem* item = parentItem->child(row);
    if (item) {
        return createIndex(row, column, item);
    } else
        return QModelIndex();
}

// 创建index的父索引
QModelIndex TreeModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return QModelIndex();

    TreeItem* item = itemFromIndex(index);
    TreeItem* parentItem = item->parent();

    if (parentItem == m_rootItem) {
        return QModelIndex();
    }
    return createIndex(parentItem->row(), 0, parentItem);
}

// 获取索引parent下有多少行
int TreeModel::rowCount(const QModelIndex& parent) const
{
    if (parent.column() > 0)
        return 0;

    TreeItem* item = itemFromIndex(parent);
    if (item) {
        return item->childCount();
    }

    return 0;
}

// 返回索引parent下有多少列
int TreeModel::columnCount(const QModelIndex& parent) const
{
    return m_header.size();
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal) {
        if (role == Qt::DisplayRole) {
            return m_header.at(section);
        }
    }
    return QVariant();
}

void TreeModel::addChildItems(const QModelIndex& parentIndex, TreeItem* childItem)
{
    auto parentItem = childItem->parent();
    if (!parentItem) {
        return;
    }

    int first = parentItem->childCount();

    beginInsertRows(parentIndex, first, first);

    parentItem->addChild(childItem);

    endInsertRows();
}

void TreeModel::removeChildItem(const QModelIndex& parentIndex, TreeItem* childItem, PModel model)
{
    auto parentItem = childItem->parent();
    if (!parentItem) {
        return;
    }

    int first = childItem->row();

    beginRemoveRows(parentIndex, first, first);

    parentItem->removeChild(childItem, model);

    endRemoveRows();
}

void TreeModel::removeChildrenItems(const QModelIndex& parentIndex, TreeItem* parentItem, PModel model)
{
    if (!parentItem) {
        return;
    }
    if (parentItem->childCount() <= 0) {
        return;
    }

    beginRemoveRows(parentIndex, 0, parentItem->childCount() - 1);

    parentItem->removeChildren(model);

    endRemoveRows();
}

void TreeModel::removeGroup(const QModelIndex& parentIndex, TreeItem* childItem, PModel model)
{
    auto parentItem = childItem->parent();
    if (!parentItem) {
        return;
    }

    int first = childItem->row();

    beginRemoveRows(parentIndex.parent(), first, first);

    parentItem->removeGroup(childItem, model);

    endRemoveRows();
}

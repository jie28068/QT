#ifndef DEVELOPERMODEWIDGETTREEMODE_H
#define DEVELOPERMODEWIDGETTREEMODE_H

#include "../item/DeveloperModeWidgetTreeItem.h"
#include <QAbstractItemModel>
#include <QSortFilterProxyModel>

class TreeModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit TreeModel(QObject* parent = nullptr);
    ~TreeModel() override;
    /// @brief 获取根节点
    /// @return
    TreeItem* rootItem();
    /// @brief 获取选中的节点
    /// @param index
    /// @return
    TreeItem* itemFromIndex(const QModelIndex& index) const;
    /// @brief 添加组/变量
    /// @param parentIndex
    /// @param childItem
    /// @param model
    void addChildItems(const QModelIndex& parentIndex, TreeItem* childItem);
    /// @brief 删除变量
    /// @param parentIndex
    /// @param childItem
    /// @param model
    void removeChildItem(const QModelIndex& parentIndex, TreeItem* childItem, PModel model);
    /// @brief 删除组内所有变量
    /// @param parentIndex
    /// @param parentItem
    void removeChildrenItems(const QModelIndex& parentIndex, TreeItem* parentItem, PModel model);
    /// @brief 删除组
    /// @param parentIndex
    /// @param childItem
    /// @param model
    void removeGroup(const QModelIndex& parentIndex, TreeItem* childItem, PModel model);

protected:
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

private:
    /// @brief 根节点
    TreeItem* m_rootItem;
    /// @brief 表头
    QStringList m_header;
};
#endif // TREEMODEL_H

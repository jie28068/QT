#ifndef CONTROLDIALOGITEM_H
#define CONTROLDIALOGITEM_H
#ifndef TREEITEM_H
#define TREEITEM_H

#include <QVariant>
#include <QVector>

struct ControlModel;
class TreeItem
{
public:
    explicit TreeItem(const QVector<QVariant> &data, TreeItem *parent = nullptr);
    ~TreeItem();

    TreeItem *child(int number);
    int childCount() const;
    int columnCount() const;
    QVariant data(int column) const;
    bool insertChildren(int position, int count, int columns);
    bool insertColumns(int position, int columns);
    TreeItem *parent();
    bool removeChildren(int position, int count);
    bool removeColumns(int position, int columns);
    int childNumber() const;
    bool setData(int column, const QVariant &value);

private:
    QVector<TreeItem *> childItems;
    QVector<QVariant> itemData;
    QVector<ControlModel *> itemDatas;
    TreeItem *parentItem;
};
#endif // TREEITEM_H

#endif
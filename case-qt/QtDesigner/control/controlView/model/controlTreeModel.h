#ifndef CONTROLTREEMODEL_H
#define CONTROLTREEMODEL_H

#include <QAbstractItemModel>

class FloatTreeItem;

class FloatTreeModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit FloatTreeModel(const QStringList &headers, QObject *parent = nullptr);
    ~FloatTreeModel() override;

    FloatTreeItem *root();

    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QModelIndex index(int row, int column, const QModelIndex &parent) const override;
    QModelIndex parent(const QModelIndex &index) const override;
    int rowCount(const QModelIndex &parent) const override;
    int columnCount(const QModelIndex &parent) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;

private:
    FloatTreeItem *itemFromIndex(const QModelIndex &index) const;

private:
    QStringList _headers;
    FloatTreeItem *_rootItem;
};

#endif
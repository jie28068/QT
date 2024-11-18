#pragma once
#include <QStyledItemDelegate>
#include <QSortFilterProxyModel>

class CarTreeView;
class TarTreeDelegate : public QStyledItemDelegate
{
    Q_OBJECT
public:
    explicit TarTreeDelegate(QObject *parent = nullptr);

    // 编辑相关的接口
    QWidget *createEditor(QWidget *parent,
                          const QStyleOptionViewItem &option,
                          const QModelIndex &index) const override;

    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor,
                      QAbstractItemModel *model,
                      const QModelIndex &index) const override;

    void updateEditorGeometry(QWidget *editor,
                              const QStyleOptionViewItem &option,
                              const QModelIndex &index) const override;
};

class TreeViewProxyModel : public QSortFilterProxyModel
{
public:
    explicit TreeViewProxyModel(QObject *parent = nullptr);
    void setFilterString(const QString &strFilter = QString());
    void begin() { beginResetModel(); }
    void end() { endResetModel(); }
    void expandFilteredNodes(const QModelIndex &parent = QModelIndex());

protected:
    bool filterAcceptsRow(int source_row, const QModelIndex &source_parent) const override;

private:
    QString m_strFilterString;
    CarTreeView *m_treeView;
};

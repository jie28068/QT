#ifndef DEVELOPERMODEWIDGETTABLEMODE
#define DEVELOPERMODEWIDGETTABLEMODE

#include "../item/DeveloperModeWidgetTreeItem.h"
#include "KLModelDefinitionCore.h"
#include "PublicDefine.h"

#include <QAbstractTableModel>
#include <QComboBox>
#include <QLineEdit>
#include <QSortFilterProxyModel>
#include <QStyledItemDelegate>

using namespace Kcc::BlockDefinition;
struct DeveloperTableItem {
    DeveloperTableItem(const QString& key, const QVariant& value) : m_key(key), m_value(value) { }

    QString m_key;
    QVariant m_value;
};

class TableModel : public QAbstractTableModel
{
public:
    enum Cloum
    {
        Key = 0,
        Value
    };
    TableModel(QObject* parent = nullptr);
    ~TableModel();
    void addRow(const QString& key, const QVariant& value);
    void cleraData();
    bool removeRow(int row, const QModelIndex& parent = QModelIndex());
    void setTreeItem(TreeItem* item) { m_item = item; }

protected:
    virtual int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    virtual int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;

private slots:
    void onDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
                       const QVector<int>& roles = QVector<int>());

private:
    /// @brief 数据链表
    QList<DeveloperTableItem*> m_data;
    /// @brief 表头
    QStringList m_listHeader;
    TreeItem* m_item;
};
#endif
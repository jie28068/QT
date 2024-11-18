#ifndef CONTROLTOOLMODEL_H
#define CONTROLTOOLMODEL_H

#include <QAbstractItemModel>
#include <QList>

struct controlToolItem
{
    explicit controlToolItem(const QString &name, const QString &type, const QVariant &pixmap = QVariant()) : m_name(name), m_type(type), m_pixmap(pixmap) {}
    controlToolItem() {}
    controlToolItem(const controlToolItem &other)
    {
        m_name = other.m_name;
        m_type = other.m_type;
        m_pixmap = other.m_pixmap;
    }
    QString m_name;    // 名称
    QString m_type;    // 类型
    QVariant m_pixmap; // 图标
};

class ControlToolModel : public QAbstractListModel
{
public:
    explicit ControlToolModel(const QString &type, QObject *parent = 0);
    ~ControlToolModel();
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    QMimeData *mimeData(const QModelIndexList &indexes) const override;
    QStringList mimeTypes() const override;
    int rowCount(const QModelIndex &parent) const override;
    Qt::DropActions supportedDropActions() const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

public:
    void addItem(const QString &name, const QString &type, const QVariant &pixmap = QVariant());

private:
    QList<controlToolItem *> m_items;
    QString m_type;
};
#endif
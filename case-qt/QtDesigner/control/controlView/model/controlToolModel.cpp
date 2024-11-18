#include "controlToolModel.h"
#include "globalDefinition.h"

#include <QMimeData>
#include <QPixmap>
#include <QIcon>
ControlToolModel::ControlToolModel(const QString &type, QObject *parent) : QAbstractListModel(parent), m_type(type)
{
}

ControlToolModel::~ControlToolModel()
{
}

Qt::ItemFlags ControlToolModel::flags(const QModelIndex &index) const
{
    if (index.isValid())
        return (QAbstractListModel::flags(index) | Qt::ItemIsDragEnabled); // 可拖拽

    return Qt::ItemIsDropEnabled; // 接受拖拽
}

QMimeData *ControlToolModel::mimeData(const QModelIndexList &indexes) const
{
    QMimeData *mimeData = new QMimeData();
    // 保存数据的形式
    QByteArray encodedData;
    QDataStream stream(&encodedData, QIODevice::WriteOnly);
    for (const QModelIndex &index : indexes)
    {
        if (index.isValid())
        {
            QString name = data(index, GlobalDefinition::controlModelName).toString();
            QPixmap pixmap = qvariant_cast<QPixmap>(data(index, GlobalDefinition::controlModelPixmap));
            QString type = data(index, GlobalDefinition::controlModelType).toString();
            stream << name << pixmap << type;
        }
    }

    mimeData->setData(GlobalDefinition::controlMimeType, encodedData);
    return mimeData;
}

QStringList ControlToolModel::mimeTypes() const
{
    QStringList types;
    types << GlobalDefinition::controlMimeType;
    return types;
}

int ControlToolModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : m_items.size();
}

Qt::DropActions ControlToolModel::supportedDropActions() const
{
    return Qt::MoveAction | Qt::CopyAction;
}

QVariant ControlToolModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
    {
        return QVariant();
    }

    if (role == Qt::DecorationRole) // 返回图标,通过pixmaps映射返回对应索引的图标,并对其进行缩放
    {
        auto pixmap = m_items.value(index.row())->m_pixmap.value<QPixmap>();
        if (!pixmap.isNull())
        {
            return QIcon(pixmap.scaled(GlobalDefinition::imageWidth, GlobalDefinition::imageHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    }
    else if (role == GlobalDefinition::controlModelName)
    {
        return m_items.value(index.row())->m_name;
    }
    else if (role == GlobalDefinition::controlModelPixmap)
    {
        return m_items.value(index.row())->m_pixmap;
    }
    else if (role == GlobalDefinition::controlModelType)
    {
        return m_items.value(index.row())->m_type;
    }
    else if (Qt::ToolTipRole == role)
    {
        return m_items.value(index.row())->m_name;
    }

    return QVariant();
}

void ControlToolModel::addItem(const QString &name, const QString &type, const QVariant &pixmap)
{
    auto isExist = [&](const QString &name) -> bool
    {
        for (auto item : m_items)
        {
            if (item->m_name == name)
            {
                return true;
            }
        }
        return false;
    };
    if (!isExist(name))
    {
        beginInsertRows(QModelIndex(), m_items.size(), m_items.size());
        m_items.append(new controlToolItem(name, type, pixmap));
        endInsertRows();
    }
}

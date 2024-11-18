#include "controlToolListModel.h"

IconTextModel::IconTextModel(QObject *parent)
    : QAbstractListModel(parent) {}

int IconTextModel::rowCount(const QModelIndex &parent) const
{
    return m_data.count();
}

int IconTextModel::columnCount(const QModelIndex &parent) const
{
    return 2;
}

QVariant IconTextModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_data.count())
        return QVariant();

    switch (role)
    {
    case Qt::DecorationRole:
        return QIcon(m_data.at(index.row()).m_icon);
    case Qt::DisplayRole:
        return m_data.at(index.row()).m_name;
    case Qt::UserRole:
        return m_data.at(index.row()).m_isChecked;
    default:
        return QVariant();
    }
}

bool IconTextModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::UserRole)
    {
        m_data[index.row()].m_isChecked = value.toBool();
        emit dataChanged(index, index);
    }
    return true;
}

Qt::ItemFlags IconTextModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable;
}

void IconTextModel::addData(const QString &text, const QString &icon, bool isChecked)
{
    m_data.append(ModelData(text, isChecked, icon));
}
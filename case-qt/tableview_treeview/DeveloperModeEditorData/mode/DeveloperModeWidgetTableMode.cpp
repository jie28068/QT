#include "DeveloperModeWidgetTableMode.h"
#include <QPainter>
#include <QSvgRenderer>
TableModel::TableModel(QObject* parent) : QAbstractTableModel(parent)
{
    m_listHeader = QStringList() << tr("键") << tr("值");
    connect(this, &QAbstractTableModel::dataChanged, this, &TableModel::onDataChanged);
}

TableModel::~TableModel() { }

void TableModel::addRow(const QString& key, const QVariant& value)
{
    // 不添加相同的key
    auto iscut = [=]() -> bool {
        for (auto var : m_data) {
            if (var->m_key == key && key != Developer::newKey) {
                return false;
            }
        }
        return true;
    };
    if (iscut()) {
        beginInsertRows(QModelIndex(), m_data.size(), m_data.size());
        m_data.append(new DeveloperTableItem(key, value));
        endInsertRows();
    }
}

void TableModel::cleraData()
{
    beginResetModel(); // 通知视图即将重置模型
    m_data.clear();    // 清空数据存储
    endResetModel();   // 通知视图模型已经重置
}

int TableModel::rowCount(const QModelIndex& parent) const
{
    return m_data.count();
}

int TableModel::columnCount(const QModelIndex& parent) const
{
    return m_listHeader.count();
}

QVariant TableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid()) {
        return QVariant();
    }
    int row = index.row();
    int col = index.column();
    if (role == Qt::DisplayRole) {
        switch (col) {
        case Key: {
            auto str = Kcc::BlockDefinition::RoleDataDefinition::index2DataDef(m_data[row]->m_key).desc();
            if (str != "unknow") {
                return str;
            } else {
                return m_data[row]->m_key;
            }
        }
        case Value:
            return m_data[row]->m_value;
        default:
            return QVariant();
        }
    } else if (role == Qt::DecorationRole) {
        if (col == Value) {
            auto var = m_data[row]->m_value;
            QImage image = qvariant_cast<QImage>(var);
            if (!image.isNull()) {
                QPixmap pixmap(QPixmap::fromImage(image));
                return QPixmap(pixmap);
            } else {
                QByteArray svgData = var.toByteArray();
                QSvgRenderer renderer(svgData);
                QPixmap pixmap(renderer.defaultSize());
                pixmap.fill(Qt::transparent); // 确保背景透明
                // 创建 QPainter 并绘制 SVG 图像
                QPainter painter(&pixmap);
                renderer.render(&painter);
                painter.end(); // 渲染完成
                return QPixmap(pixmap);
            }
        }
    }
    return QVariant();
}

bool TableModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (!index.isValid()) {
        return QAbstractTableModel::setData(index, value, role);
    }
    int colum = index.column();
    int row = index.row();
    if (role == Qt::EditRole) {
        switch (colum) {
        case Key: {
            auto str = QString(Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(value.toString()));
            if (str != "-1") {
                m_data[row]->m_key = str;
            } else {
                m_data[row]->m_key = value.toString();
            }
            emit dataChanged(index, index);
        } break;
        case Value: {
            m_data[row]->m_value = value;
            emit dataChanged(index, index);
        } break;
        default:
            break;
        }
        return true;
    }
    return QAbstractTableModel::setData(index, value, role);
}

QVariant TableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal) {
        if (Qt::DisplayRole == role) {
            Q_ASSERT(section < m_listHeader.count());
            return m_listHeader[section];
        }
    }
    return QAbstractTableModel::headerData(section, orientation, role);
}

Qt::ItemFlags TableModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    auto keyName = index.siblingAtColumn(0).data().toString();
    if (index.column() == Key && index.data().toString() == Developer::isSharedGroup) { // 不可修改同步的key
        flags |= Qt::ItemIsSelectable;
    } else if (keyName == Developer::uuid) { // 不可修改uuid
        flags |= Qt::ItemIsSelectable;
    } else if (index.column() == Value
               && (Developer::PNGImageLists.contains(Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(keyName))
                   || Developer::SVGImageLists.contains(
                           Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(keyName)))) { // 不可双击编辑图片
        flags |= Qt::ItemIsSelectable;
    } else {
        flags |= Qt::ItemIsEditable;
    }
    return flags;
}

bool TableModel::removeRow(int row, const QModelIndex& parent)
{
    beginRemoveRows(parent, row, row);
    if (m_item->m_type == TreeItem::LeafNode) {
        if (m_item->getVarable()) {
            m_item->getVarable()->removeRole(m_data[row]->m_key);
        }
    } else if (m_item->m_type == TreeItem::BranchNode) {
        if (m_item->getVarableGroup()) {
            m_item->getVarableGroup()->removeRole(m_data[row]->m_key);
        }
    }
    m_data.removeAt(row);
    endRemoveRows();
    return true;
}

void TableModel::onDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight, const QVector<int>& roles)
{
    if (m_item->m_type == TreeItem::LeafNode) {
        if (m_item->getVarable()) {
            if (m_data[topLeft.row()]->m_key != Developer::newKey) {
                QPointF point; // 由于端口位置是point类型，要特殊处理才能保存
                if (m_data[topLeft.row()]->m_key == Kcc::BlockDefinition::RoleDataDefinition::PortPresetPosition) {
                    QStringList strList = m_data[topLeft.row()]->m_value.toString().split(",");
                    double num1 = 0;
                    double num2 = 0;
                    if (strList.size() >= 2) {
                        num1 = strList[0].remove(0, 1).toDouble();
                        strList[1].chop(1);
                        num2 = strList[1].toDouble();
                    }
                    point = (QPointF(num1, num2));
                }
                m_item->getVarable()->setData(m_data[topLeft.row()]->m_key,
                                              point.isNull() ? m_data[topLeft.row()]->m_value : point);
            }
        }
    } else if (m_item->m_type == TreeItem::BranchNode) {
        if (m_item->getVarableGroup()) {
            if (m_data[topLeft.row()]->m_key == Developer::isSharedGroup) {
                m_item->getVarableGroup()->setSameVariableCountWithOrigin(
                        m_data[topLeft.row()]->m_value == "true" ? true : false);
            } else {
                if (m_data[topLeft.row()]->m_key != Developer::newKey) {
                    m_item->getVarableGroup()->setGroupData(m_data[topLeft.row()]->m_key,
                                                            m_data[topLeft.row()]->m_value);
                }
            }
        }
    }
}
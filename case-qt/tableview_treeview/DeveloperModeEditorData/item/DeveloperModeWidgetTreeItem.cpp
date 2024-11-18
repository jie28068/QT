#include "DeveloperModeWidgetTreeItem.h"

TreeItem::TreeItem(QString uuid, Type type, QString uuidname, TreeItem* parent)
    : m_uuid(uuid), m_type(type), m_parent(parent)
{
    m_row = 0;
    _ptr = new Node;
    _ptr->Node = uuid;
    switch (type) {
    case HeadNode:
        _ptr->Type = ItemType::otherTypeItem;
        break;
    case BranchNode:
        _ptr->Type = ItemType::groupTypeItem;
        break;
    case LeafNode:
        _ptr->Type = ItemType::variableTypeItem;
        break;
    default:
        break;
    }
    if (uuidname != "") {
        m_uuidName[uuid] = uuidname;
    }
}

QVariant TreeItem::data(int column) const
{
    Node* p = (Node*)_ptr;
    if (p) {
        switch (column) {
        case COLUMN_NODE: {
            if (m_type == LeafNode) {
                if (m_uuidName[m_uuid] != "")
                    return m_uuidName[m_uuid];
                else
                    return m_uuid;
            } else {
                return p->Node;
            }
        }
        case COLUMN_TYPE:
            return p->Type;
        default:
            return m_uuid;
        }
    }
    return QVariant();
}

TreeItem::~TreeItem() { }

// 在本节点下添加子节点
void TreeItem::addChild(TreeItem* item)
{
    item->setRow(m_children.size());
    m_children.append(item);
}

void TreeItem::removeChild(TreeItem* item, PModel m_model)
{
    // 删除model中的数据
    if (item && m_model && m_variableGroup && item->m_variable) {
        m_variableGroup->removeVariable(item->m_variable->getUUID());
    }
    //  删除子item
    m_children.removeOne(item);
    delete item;
    item = nullptr;
    // 重新设置item的Row
    for (int i = 0; i < m_children.size(); i++) {
        m_children[i]->setRow(i);
    }
}

// 清空所有子节点
void TreeItem::removeChildren(PModel model)
{
    // 删除model中的数据
    if (m_variableGroup) {
        for (auto var : m_variableGroup->getVariableMap().toStdMap()) {
            m_variableGroup->removeVariable(var.second->getUUID());
        }
    }

    qDeleteAll(m_children);
    m_children.clear();
}

void TreeItem::removeGroup(TreeItem* item, PModel model)
{
    // 删除model中的数据
    if (item && model && item->m_variableGroup) {
        model->removeVariableGroup(item->m_variableGroup->getGroupType());
    }
    //  删除子item
    m_children.removeOne(item);
    delete item;
    item = nullptr;
    // 重新设置item的Row
    for (int i = 0; i < m_children.size(); i++) {
        m_children[i]->setRow(i);
    }
}

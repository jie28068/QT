#include "controlTreeItem.h"

#include <QIcon>

FloatTreeItem::FloatTreeItem(FloatTreeItem *parent)
    : _parent(parent),
      _type(UNKNOWN),
      _ptr(nullptr),
      _row(-1),
      _checked(false)
{
}

FloatTreeItem::~FloatTreeItem()
{
    removeChildren();
}

// 在本节点下添加子节点
void FloatTreeItem::addChild(FloatTreeItem *item)
{
    item->setRow(_children.size());
    _children.append(item);
}

// 清空所有子节点
void FloatTreeItem::removeChildren()
{
    qDeleteAll(_children);
    _children.clear();
}

// 获取本节点第column列的数据
QVariant FloatTreeItem::data(int column) const
{
    if (_type == PROVINCE) // 此节点是"省份"信息
    {
        if (column == GlobalDefinition::COLUMN_NAME) // 省份，只有"name"字段，且显示在第一列
        {
            GlobalDefinition::Province *p = (GlobalDefinition::Province *)_ptr;
            return p->name;
        }
    }
    else if (_type == PERSON) // 此节点是"人口"信息
    {
        GlobalDefinition::Person *p = (GlobalDefinition::Person *)_ptr;
        switch (column)
        {
        case GlobalDefinition::COLUMN_NAME:
            return p->name;
        case GlobalDefinition::COLUMN_SEX:
            return p->sex;
        case GlobalDefinition::COLUMN_AGE:
            return QString::number(p->age);
        case GlobalDefinition::COLUMN_PHONE:
            return p->phone;
        default:
            return QVariant();
        }
    }
    return QVariant();
}

bool FloatTreeItem::checkable(int column) const
{
    if (column != GlobalDefinition::COLUMN_NAME) // 除第一列外，其他列不允许勾选
    {
        return false;
    }
    if (_type == PERSON) // 第一列中只允许"人口"勾选
    {
        return true;
    }
    return false;
}

QVariant FloatTreeItem::getIcon(int column)
{
    if (column != GlobalDefinition::COLUMN_NAME)
        return QVariant();

    if (_type == PROVINCE)
    {
        return QIcon(":/res/province.png");
    }
    else if (_type == PERSON)
    {
        return QIcon(":/res/person.png");
    }
    return QVariant();
}

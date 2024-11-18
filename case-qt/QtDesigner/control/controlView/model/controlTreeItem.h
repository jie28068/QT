#ifndef CONTROLTREEITEM_H
#define CONTROLTREEITEM_H

#include "globalDefinition.h"
#include <QVariant>

class FloatTreeItem
{
public:
    enum Type
    {
        UNKNOWN = -1,
        PROVINCE,
        PERSON
    };

    explicit FloatTreeItem(FloatTreeItem *parent = nullptr);
    ~FloatTreeItem();

    void addChild(FloatTreeItem *item);
    void removeChildren();

    FloatTreeItem *child(int row) { return _children.value(row); }
    FloatTreeItem *parent() { return _parent; }

    int childCount() const { return _children.count(); }

    QVariant data(int column) const;

    // 设置、获取节点存的数据指针
    void setPtr(void *p) { _ptr = p; }
    void *ptr() const { return _ptr; }

    // 保存该节点是其父节点的第几个子节点，查询优化所用
    void setRow(int row) { _row = row; }
    // 返回本节点位于父节点下第几个子节点
    int row() const { return _row; }

    Type getType() const { return _type; }
    void setType(const Type &value) { _type = value; }

    bool checkable(int column) const;

    bool isChecked() const { return _checked; }
    void setChecked(bool check) { _checked = check; }

    QVariant getIcon(int column);

private:
    QList<FloatTreeItem *> _children; // 子节点
    FloatTreeItem *_parent;           // 父节点
    Type _type;                       // 此节点保存的数据类型
    void *_ptr;                       // 存储数据的指针
    int _row;                         // 此item位于父节点中第几个
    bool _checked;
};

#endif
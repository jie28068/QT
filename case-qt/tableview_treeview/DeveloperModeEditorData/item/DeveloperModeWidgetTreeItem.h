#ifndef DEVELOPERMODEWIDGETTREEITEM_H
#define DEVELOPERMODEWIDGETTREEITEM_H

#include "KLModelDefinitionCore/KLModelDefinitionCore.h"
#include "typedef.h"
#include <QModelIndex>
#include <QVariant>

using namespace Kcc::BlockDefinition;
class TreeItem
{
public:
    // 节点类型
    enum Type
    {
        Unknown,
        HeadNode,   // 头节点
        BranchNode, // 分支节点
        LeafNode,   // 叶子节点
    };
    /// @brief
    /// @param name 节点名称
    /// @param type 节点类型
    /// @param uuid 节点别名
    /// @param parent 节点上一层
    explicit TreeItem(QString name = "", Type type = Unknown, QString uuid = "", TreeItem* parent = nullptr);
    ~TreeItem();

    TreeItem* parent() { return m_parent; }
    /// @brief 添加节点
    /// @param item
    void addChild(TreeItem* item);
    /// @brief 删除变量
    /// @param item
    /// @param model
    void removeChild(TreeItem* item, PModel model);
    /// @brief 删除组下所有变量
    /// @param model
    void removeChildren(PModel model);
    /// @brief 删除组
    /// @param item
    /// @param model
    void removeGroup(TreeItem* item, PModel model);

    QVariant data(int column) const;
    TreeItem* child(int row) { return m_children.value(row); }
    QList<TreeItem*> children() { return m_children; }
    void setChildren(QList<TreeItem*> children) { m_children = children; }

    int childCount() const { return m_children.count(); }

    void setRow(int row) { m_row = row; } // 保存该节点是其父节点的第几个子节点，查询优化所用
    int row() const { return m_row; }     // 返回本节点位于父节点下第几个子节点

    void setVarable(PVariable var) { m_variable = var; }
    PVariable getVarable() { return m_variable; }

    void setVarableGroup(PVariableGroup var) { m_variableGroup = var; }
    PVariableGroup getVarableGroup() { return m_variableGroup; }

public:
    QString m_uuid; // uuid
    Type m_type;    // 类型
    /// @brief uuid和name对应，用于树状显示
    QMap<QString, QString> m_uuidName;

private:
    PVariable m_variable;           // 用于表格数据-变量
    PVariableGroup m_variableGroup; // 用于表格数据-组
    QList<TreeItem*> m_children;    // 子节点
    TreeItem* m_parent;             // 父节点
    int m_row;                      // 此item位于父节点中第几个
    Node* _ptr;                     // 存储数据的指针
};

#endif
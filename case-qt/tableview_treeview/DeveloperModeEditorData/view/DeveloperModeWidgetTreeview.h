#ifndef DEVELOPERMODEWIDGETTREEVIEW_H
#define DEVELOPERMODEWIDGETTREEVIEW_H

#include "../item/typedef.h"
#include "../mode/DeveloperModeWidgetTreeMode.h"
#include "KCustomDialog.h"
#include "KLModelDefinitionCore.h"
#include "KLWidgets/KItemView.h"

#include <QFile>
#include <QHeaderView>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>

using namespace Kcc::BlockDefinition;
class TreeViewProxyModel;
class ModelTreeView : public KTreeView
{
    Q_OBJECT
public:
    ModelTreeView(PModel model, QWidget* parent);
    ~ModelTreeView();

    void initTreeView(); // 初始化

    void InitData();

    TreeViewProxyModel* getTreeViewProxyModel() { return proxymodel; }

    TreeItem* getItemByIndex(const QModelIndex& index);

    QList<std::pair<QString, PVariable>> sortVariable(QMap<QString, PVariable>& orignmap, const QString& groupname);

private slots:
    void onTreeContextMenu(const QPoint& pos); // 右键菜单
    void onAddGroup();                         // 添加组
    void onDeleteVariable();                   // 删除变量
    void onDeleteVariables();                  // 删除全部变量
    void onDeleteGroup();                      // 删除组
    void onAddVariable();                      // 添加变量
    void onOpenTree();                         // 展开所有节点
private:
    void onOpenGroupTree(); // 展开组节点

private:
    /// @brief 获取模块没有的组名称
    /// @return
    QStringList inexistenceGroupNameList();

private:
    TreeModel* m_treeModel;
    /// @brief  临时
    PModel m_model;
    QMenu* m_menu;
    TreeItem* modelNmaeItem; // 头节点
    TreeViewProxyModel* proxymodel;
};

// 排序和筛选 Model
class TreeViewProxyModel : public QSortFilterProxyModel
{
public:
    explicit TreeViewProxyModel(QObject* parent = nullptr);
    void setFilterString(const QString& strFilter = QString());

    void begin() { beginResetModel(); }
    void end() { endResetModel(); }
    /// @brief 展开筛选后的节点
    /// @param source_parent
    /// @return
    void expandFilteredNodes(const QModelIndex& parent = QModelIndex());

protected:
    bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const override;

private:
    QString m_strFilterString;
    ModelTreeView* mtree;
};

#endif
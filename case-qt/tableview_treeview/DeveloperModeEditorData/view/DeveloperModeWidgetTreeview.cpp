#include "DeveloperModeWidgetTreeview.h"
#include <QComboBox>
#include <QHBoxLayout>

ModelTreeView::ModelTreeView(PModel model, QWidget* parent)
    : KTreeView(parent), m_treeModel(nullptr), m_model(model), m_menu(new QMenu(this))
{
    this->setContextMenuPolicy(Qt::CustomContextMenu);
    InitData();
    initTreeView();
    onOpenGroupTree();
    connect(this, &ModelTreeView::customContextMenuRequested, this, &ModelTreeView::onTreeContextMenu);
}

ModelTreeView::~ModelTreeView() { }

void ModelTreeView::initTreeView()
{
    proxymodel = new TreeViewProxyModel(this);
    proxymodel->setSourceModel(m_treeModel);
    setModel(proxymodel);

    // 此函数用于调整列宽
    auto adjustColumnWidth = [=]() {
        int totalWidth = viewport()->width();                      // 获取视图的总宽度
        int firstColumnWidth = static_cast<int>(totalWidth * 0.8); // 计算出第一列的宽度
        setColumnWidth(0, firstColumnWidth);                       // 设置第一列的宽度
    };
    // 在视图第一次显示的时候可能需要调整列宽
    adjustColumnWidth();
    // 为了响应后续的尺寸变化，比如用户调整窗口大小，我们需要连接 resize 事件
    QObject::connect(header(), &QHeaderView::sectionResized, this, adjustColumnWidth);
}

QList<std::pair<QString, PVariable>> ModelTreeView::sortVariable(QMap<QString, PVariable>& orignmap,
                                                                 const QString& groupname)
{
    QList<std::pair<QString, PVariable>> resultlist;
    for (std::pair<const QString, PVariable> pair : orignmap.toStdMap()) {
        resultlist.append(pair);
    }

    if (!Developer::otherGroupNameLists.contains(groupname)) {
        std::sort(resultlist.begin(), resultlist.end(),
                  [](const std::pair<QString, PVariable>& s1, const std::pair<QString, PVariable>& s2) {
                      if (s1.second && s2.second)
                          return s1.second->getOrder() < s2.second->getOrder();
                      else
                          return false;
                  });
    }
    return resultlist;
}

void ModelTreeView::InitData()
{
    m_treeModel = new TreeModel();
    if (m_treeModel && m_model) {
        TreeItem* rootItem = m_treeModel->rootItem();
        modelNmaeItem = new TreeItem(m_model->getName(), TreeItem::HeadNode, "", rootItem);
        rootItem->addChild(modelNmaeItem);
        // 获取group
        auto groupList = m_model->getVariableGroupList();
        for (auto group : groupList) {
            QString groupType = group->getGroupType();
            TreeItem* groupNameItem = new TreeItem(groupType, TreeItem::BranchNode, "", modelNmaeItem);
            groupNameItem->setVarableGroup(group); // 设置分组
            modelNmaeItem->addChild(groupNameItem);
            // 获取variable
            QMap<QString, PVariable> groupMap = group->getVariableMap();
            auto variablelist = sortVariable(groupMap, groupType);
            for (auto pair : variablelist) {
                // 为了树状显示，uuid使用组内的名称替代
                TreeItem* variableItem;
                if (Developer::otherGroupNameLists.contains(groupType)) {
                    variableItem = new TreeItem(
                            pair.first, TreeItem::LeafNode,
                            pair.second->getData(Kcc::BlockDefinition::RoleDataDefinition::VariableTypeRole).toString(),
                            groupNameItem);
                } else {
                    variableItem = new TreeItem(pair.first, TreeItem::LeafNode, pair.second->getName(), groupNameItem);
                }
                variableItem->setVarable(pair.second); // 设置变量
                variableItem->setVarableGroup(group);  // 设置分组
                groupNameItem->addChild(variableItem);
            }
        }
    }
}

TreeItem* ModelTreeView::getItemByIndex(const QModelIndex& index)
{
    auto mapIndex = proxymodel->mapToSource(index);
    return m_treeModel->itemFromIndex(mapIndex);
}

void ModelTreeView::onAddGroup()
{
    auto index = currentIndex();
    auto nindex = proxymodel->mapToSource(index);
    TreeItem* item = getItemByIndex(index);
    if (item) {
        QWidget* widget = new QWidget(this);
        QComboBox* combox = new QComboBox(widget);
        combox->addItems(inexistenceGroupNameList());
        QHBoxLayout* layout = new QHBoxLayout(widget);
        layout->addWidget(combox);
        widget->setLayout(layout);
        KCustomDialog dlg(tr("添加新组"), widget, KBaseDlgBox::StandardButton::Ok | KBaseDlgBox::StandardButton::Cancel,
                          KBaseDlgBox::StandardButton::Ok, this);
        dlg.resize(400, 200);
        if (KBaseDlgBox::Ok == dlg.exec()) {
            auto gname = combox->currentText();
            TreeItem* groupNameItemNem = new TreeItem(gname, TreeItem::BranchNode, "", modelNmaeItem);
            m_treeModel->addChildItems(nindex, groupNameItemNem);
            groupNameItemNem->setVarableGroup(m_model->createVariableGroup(gname));
        }
    }
}

void ModelTreeView::onDeleteVariable()
{
    auto index = currentIndex();
    auto nindex = proxymodel->mapToSource(index.parent());
    TreeItem* item = getItemByIndex(index);
    if (item) {
        m_treeModel->removeChildItem(nindex, item, m_model);
    }
}

void ModelTreeView::onDeleteVariables()
{
    auto index = currentIndex();
    auto nindex = proxymodel->mapToSource(index);
    TreeItem* item = getItemByIndex(index);
    if (item) {
        m_treeModel->removeChildrenItems(nindex, item, m_model);
    }
}

void ModelTreeView::onDeleteGroup()
{
    auto index = currentIndex();
    auto nindex = proxymodel->mapToSource(index);
    TreeItem* item = getItemByIndex(index);
    if (item) {
        m_treeModel->removeGroup(nindex, item, m_model);
    }
}

void ModelTreeView::onAddVariable()
{
    auto index = currentIndex();
    auto nindex = proxymodel->mapToSource(index);
    TreeItem* item = getItemByIndex(index);
    if (item && item->getVarableGroup()) {
        auto variable = item->getVarableGroup()->createVariable();
        TreeItem* ItemNew = new TreeItem(variable->getUUID(), TreeItem::LeafNode, "", item);
        ItemNew->setVarable(variable);
        ItemNew->setVarableGroup(item->getVarableGroup());
        m_treeModel->addChildItems(nindex, ItemNew);
    }
}

void ModelTreeView::onOpenTree()
{
    expandAll();
}

void ModelTreeView::onOpenGroupTree()
{
    for (int i = 0; i < proxymodel->rowCount(); i++) {
        QModelIndex index = proxymodel->index(i, 0);
        expand(index);
    }
}

QStringList ModelTreeView::inexistenceGroupNameList()
{
    auto groupList = m_model->getVariableGroupList();
    QStringList currntList;
    for (auto var : groupList) {
        currntList.append(var->getGroupType());
    }
    QStringList newList;
    QStringList allList;
    if (m_model->getModelType() == Model::Control_Block_Type) {
        allList = Developer::controlGroupNameLists;
    } else if (m_model->getModelType() == Model::Elec_Block_Type) {
        allList = Developer::elecGroupNameLists;
    } else if (m_model->getModelType() == Model::Device_Model_Type) {
        allList = Developer::deviceGroupNameLists;
    } else {
        allList = Developer::otherGroupNameList;
    }
    for (auto var : allList) {
        if (!currntList.contains(var)) {
            newList.append(var);
        }
    }
    return newList;
}

void ModelTreeView::onTreeContextMenu(const QPoint& pos)
{
    m_menu->clear();
    QModelIndex index = indexAt(pos);
    if (!index.isValid()) {
        return;
    }
    TreeItem* item = getItemByIndex(index);
    if (!item) {
        return;
    }
    switch (item->m_type) {
    case TreeItem::HeadNode: {
        m_menu->addAction("添加组", this, SLOT(onAddGroup()));
        m_menu->addAction("展开所有节点", this, SLOT(onOpenTree()));
    } break;
    case TreeItem::BranchNode: {
        TreeItem* item = getItemByIndex(index);
        m_menu->addAction("添加变量", this, SLOT(onAddVariable()));
        // 这俩个组不可删除
        if (item->getVarableGroup()) {
            if (item->getVarableGroup()->getGroupType() != Kcc::BlockDefinition::RoleDataDefinition::PortGroup
                && item->getVarableGroup()->getGroupType() != Kcc::BlockDefinition::RoleDataDefinition::MainGroup) {
                m_menu->addAction("删除组", this, SLOT(onDeleteGroup()));
            }
        }
        m_menu->addAction("删除全部变量", this, SLOT(onDeleteVariables()));
    } break;
    case TreeItem::LeafNode: {
        m_menu->addAction("删除变量", this, SLOT(onDeleteVariable()));
    } break;
    default:
        break;
    }
    m_menu->exec(cursor().pos());
}

TreeViewProxyModel::TreeViewProxyModel(QObject* parent) : QSortFilterProxyModel(parent)
{
    mtree = qobject_cast<ModelTreeView*>(parent);
}

void TreeViewProxyModel::expandFilteredNodes(const QModelIndex& parent)
{
    int rowCount = mtree->model()->rowCount(parent);
    for (int i = 0; i < rowCount; ++i) {
        QModelIndex child = mtree->model()->index(i, 0, parent);
        // 仅对当前没有展开的节点进行处理
        if (mtree->model()->hasChildren(child) && !mtree->isExpanded(child)) {
            mtree->expand(child);
        }
        // 递归
        expandFilteredNodes(child);
    }
}

void TreeViewProxyModel::setFilterString(const QString& strFilter)
{
    m_strFilterString = strFilter;
    invalidateFilter();
    // 遍历筛选后的节点并展开
    expandFilteredNodes();
}

bool TreeViewProxyModel::filterAcceptsRow(int source_row, const QModelIndex& source_parent) const
{
    QModelIndex index = sourceModel()->index(source_row, 0, source_parent);
    QString strDisplayName = index.data(Qt::DisplayRole).toString();
    TreeModel* treeModel = qobject_cast<TreeModel*>(sourceModel());
    if (!treeModel) {
        return false;
    }

    bool m_pFilter = QObject::tr(strDisplayName.toUtf8()).contains(m_strFilterString, Qt::CaseInsensitive)
            || strDisplayName.contains(m_strFilterString, Qt::CaseInsensitive);
    if (m_pFilter) {
        return true;
    } else {
        // 不符合条件的父节点 对其子节点进行递归判断处理
        for (int i = 0; i < sourceModel()->rowCount(index); i++) {
            if (TreeViewProxyModel::filterAcceptsRow(i, index)) {
                return true;
            }
        }
        return false;
    }
}
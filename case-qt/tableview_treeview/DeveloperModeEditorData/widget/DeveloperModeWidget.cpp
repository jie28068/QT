#include "DeveloperModeWidget.h"
#include "WizardServerMng.h"
#include <QHeaderView>

DeveloperModeWidget::DeveloperModeWidget(PModel model, QWidget* parent) : QMainWindow(parent)
{
    dataPtr.reset(new DeveloperModeWidgetPrivate());
    dataPtr->m_model = model;
    dataPtr->tempModel = dataPtr->m_model->copy(true);
    InitUI();
}

DeveloperModeWidget::~DeveloperModeWidget() { }

void DeveloperModeWidget::InitUI()
{
    QWidget* treewidget = new QWidget(this);
    auto* splitter = new QSplitter(this);
    QVBoxLayout* vLayout = new QVBoxLayout(treewidget);
    // 创建树形视图
    dataPtr->treeView = new ModelTreeView(dataPtr->tempModel, treewidget);
    // 创建代理模型
    dataPtr->tableModel = new TableModel();
    // 创建表格视图
    dataPtr->tableView = new TableView(dataPtr->tableModel, this);

    SearchLineEdit* textBox = new SearchLineEdit(dataPtr->treeView, treewidget);
    vLayout->addWidget(textBox, 10);
    vLayout->addWidget(dataPtr->treeView, 5);
    treewidget->setLayout(vLayout);
    splitter->addWidget(treewidget);
    splitter->addWidget(dataPtr->tableView);
    setCentralWidget(splitter);

    // 连接树的信号到插槽，以对项目点击作出响应
    connect(dataPtr->treeView, &QTreeView::clicked, this, &DeveloperModeWidget::onTreeClicked);
}

void DeveloperModeWidget::save()
{
    QStringList lists;
    for (auto name : dataPtr->tempModel->getVariableGroupList()) {
        lists.append(name->getGroupType());
    }
    saveData(lists);
    if (Model::Device_Model_Type == dataPtr->tempModel->getModelType()) {
        BuiltinModelManager::getInstance().getBuiltinProject()->saveModel(dataPtr->m_model);
    } else {
        WizardServerMng::getInstance().m_pModelServer->SaveToolkitModel(dataPtr->m_model);
    }
}

void DeveloperModeWidget::setTabelData(TreeItem* item)
{
    dataPtr->tableModel->setTreeItem(item);
    if (item->m_type == TreeItem::LeafNode) {
        if (item->getVarable()) {
            auto muuid = item->getVarable()->getUUID();
            if (!muuid.isEmpty())
                dataPtr->tableModel->addRow(Developer::uuid, muuid); // 添加uuid
            auto map = item->getVarable()->getRoleData();
            for (auto v : map.toStdMap()) {
                QString str; // 由于端口位置是point类型，要特殊处理才会显示在界面上
                if (v.first == Kcc::BlockDefinition::RoleDataDefinition::PortPresetPosition) {
                    str = QString("(%1,%2)").arg(v.second.toPointF().x()).arg(v.second.toPointF().y());
                }
                dataPtr->tableModel->addRow(v.first, str.isEmpty() ? v.second : str);
            }
        }
    } else if (item->m_type == TreeItem::BranchNode) {
        if (item->getVarableGroup()) {
            // 特殊字段
            dataPtr->tableModel->addRow(Developer::isSharedGroup,
                                        item->getVarableGroup()->isSameVariableCountWithOrigin());
            for (auto var : item->getVarableGroup()->getGroupDataMap().toStdMap()) {
                if (var.first != Kcc::BlockDefinition::RoleDataDefinition::GroupTypeRole) {
                    dataPtr->tableModel->addRow(var.first, var.second);
                }
            }
        }
    }
}

void DeveloperModeWidget::saveData(const QStringList& groupnamelist)
{
    // 保存未修改前的组名
    auto oldgrouplists = dataPtr->m_model->getVariableGroupList();
    QStringList oldgroupnames;
    for (auto var : oldgrouplists) {
        oldgroupnames.append(var->getGroupType());
    }
    // 保存数据
    for (auto name : groupnamelist) {
        saveGroupData(dataPtr->m_model, dataPtr->tempModel, name);
    }
    // 删除被删除的组
    QSet<QString> setA = QSet<QString>::fromList(oldgroupnames);
    QSet<QString> setB = QSet<QString>::fromList(groupnamelist);
    QSet<QString> uniqueInA = setA - setB;
    QStringList result = QStringList::fromSet(uniqueInA);
    for (auto var : result) {
        dataPtr->m_model->removeVariableGroup(var);
    }
}

void DeveloperModeWidget::saveGroupData(PModel model, PModel tempModel, const QString& groupname)
{
    if (model && tempModel) {
        PVariableGroup Group = tempModel->getVariableGroup(groupname);
        if (Group) {
            copyVariable(Group, groupname, model);
        } else {
            model->removeVariableGroup(groupname);
        }
    }
}

void DeveloperModeWidget::onTreeClicked(const QModelIndex& index)
{
    // 根据index获取被点击的项
    auto item = dataPtr->treeView->getItemByIndex(index);
    if (item && item->parent()) {
        dataPtr->tableModel->cleraData();
        setTabelData(item);
        dataPtr->tableView->update(); // 刷新表格视图
    }
}

void DeveloperModeWidget::copyVariable(PVariableGroup groupValue, const QString& str, PModel model)
{
    if (model) {
        PVariableGroup currntgroup = model->getVariableGroup(str);
        if (!currntgroup) {
            currntgroup = model->createVariableGroup(str);
        }
        // 拷贝组内数据
        currntgroup->setSameVariableCountWithOrigin(groupValue->isSameVariableCountWithOrigin());
        for (auto var : groupValue->getGroupDataMap().toStdMap()) {
            currntgroup->setGroupData(var.first, var.second);
        }
        for (auto variable : groupValue->getVariableMap()) {
            auto originalVariable = currntgroup->getVariable(variable->getUUID());
            if (originalVariable) {
                copyVariableData(originalVariable, variable);
            } else {
                auto newVariable =
                        currntgroup->createVariable([=](PVariable var) { var->setUUID(variable->getUUID()); });
                copyVariableData(newVariable, variable);
            }
        }
        if (currntgroup) {
            for (auto& uuid : currntgroup->getVariableMap().keys()) {
                if (!groupValue->hasVariable(uuid)) {
                    currntgroup->removeVariable(uuid);
                }
            }
        }
    }
}

void DeveloperModeWidget::copyVariableData(PVariable variable, PVariable tempVariable)
{
    auto tempValueMap = tempVariable->getRoleData();
    auto currntValueMap = variable->getRoleData();
    for (auto ver : currntValueMap.toStdMap()) {
        if (!tempValueMap.contains(ver.first)) {
            variable->removeRole(ver.first);
        }
    }
    for (auto ver : tempValueMap.toStdMap()) {
        variable->setData(ver.first, ver.second);
    }
}

SearchLineEdit::SearchLineEdit(ModelTreeView* treeview, QWidget* parent /* = 0 */)
    : KLineEdit(parent), m_treeview(treeview)
{
    QHBoxLayout* mainLayout = new QHBoxLayout;
    mainLayout->setMargin(0);
    mainLayout->setSpacing(0);

    QPushButton* iconButton = new QPushButton(this);
    iconButton->setObjectName("iconButton");
    iconButton->setFlat(true);

    mainLayout->addWidget(iconButton);
    mainLayout->addStretch();
    mainLayout->setContentsMargins(2, 0, 2, 0);
    setTextMargins(15, 0, 20, 0);
    setLayout(mainLayout);
    setPlaceholderText("搜索");
    connect(this, &QLineEdit::textChanged, this,
            [&](const QString& str) { m_treeview->getTreeViewProxyModel()->setFilterString(str); });
}
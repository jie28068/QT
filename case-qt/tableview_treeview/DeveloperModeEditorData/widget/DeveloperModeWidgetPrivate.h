#ifndef DEVELOPERMODEWIDGETPRIVATE
#define DEVELOPERMODEWIDGETPRIVATE

#include "DeveloperModeWidget.h"
#include <QStandardItemModel>
#include <QTableView>
#include <QTreeView>

using namespace Kcc::BlockDefinition;
class DeveloperModeWidgetPrivate
{
public:
    DeveloperModeWidgetPrivate()
    {
        tableView = nullptr;
        tableModel = nullptr;
        treeView = nullptr;
    };
    ~DeveloperModeWidgetPrivate() {};

    QTableView* tableView;
    TableModel* tableModel;
    ModelTreeView* treeView;
    PModel m_model;   // 原始数据
    PModel tempModel; // 临时数据
};

#endif
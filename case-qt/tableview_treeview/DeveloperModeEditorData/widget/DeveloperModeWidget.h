#ifndef DEVELOPERMODEWIDGET_H
#define DEVELOPERMODEWIDGET_H

#include "../mode/DeveloperModeWidgetTableMode.h"
#include "../view/DeveloperModeWidgetTableview.h"
#include "../view/DeveloperModeWidgetTreeview.h"
#include "DeveloperModeWidgetPrivate.h"
#include "KLModelDefinitionCore.h"
#include "KLProject/BuiltinModelManager.h"
#include "KLWidgets/KItemView.h"
#include "KLineEdit.h"
#include "Model.h"

#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QScrollArea>
#include <QSplitter>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QTableView>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

using namespace Kcc::BlockDefinition;
class SearchLineEdit : public KLineEdit
{
    Q_OBJECT
public:
    SearchLineEdit(ModelTreeView* treeview, QWidget* parent = 0);

private:
    ModelTreeView* m_treeview;
};

class DeveloperModeWidget : public QMainWindow
{
    Q_OBJECT
public:
    DeveloperModeWidget(PModel model, QWidget* parent = nullptr);
    ~DeveloperModeWidget();
    void InitUI();
    void save();
public slots:
    void onTreeClicked(const QModelIndex& index);

private:
    /// @brief 点击树分支时设置表格数据
    /// @param item
    void setTabelData(TreeItem* item);
    /// @brief 保存数据
    /// @param groupnamelist
    void saveData(const QStringList& groupnamelist);
    /// @brief 保存组数据
    /// @param model 原有模型数据
    /// @param tempModel 临时模型数据
    /// @param groupname 组名
    void saveGroupData(PModel model, PModel tempModel, const QString& groupname);
    /// @brief 拷贝组内的variable数据
    /// @param groupValue
    /// @param str
    /// @param model
    void copyVariable(PVariableGroup groupValue, const QString& str, PModel model);
    /// @brief 拷贝variable数据
    /// @param variable
    /// @param tempVariable
    void copyVariableData(PVariable variable, PVariable tempVariable);

private:
    QScopedPointer<DeveloperModeWidgetPrivate> dataPtr;
};
#endif

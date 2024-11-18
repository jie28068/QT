#ifndef DEVELOPERMODEWIDGETTABLEVIEW
#define DEVELOPERMODEWIDGETTABLEVIEW

#include "../delegate/DeveloperModeWidgetTableDelegate.h"
#include "../mode/DeveloperModeWidgetTableMode.h"
#include "KMessageBox.h"

#include <QAbstractItemModel>
#include <QTableView>
class TableView : public QTableView
{
public:
    TableView(TableModel* model, QWidget* parent = nullptr);
    ~TableView();

protected:
    void useImageMenu(QModelIndex index, const QString& strs);
    void inputImage(QModelIndex index, const QString& strs);

private slots:
    void showContextMenu(const QPoint& pos);

private:
    TableModel* m_model;
};

#endif
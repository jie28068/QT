#ifndef DEVELOPERMODEWIDGETTABLEDELEGATE_H
#define DEVELOPERMODEWIDGETTABLEDELEGATE_H

#include "../mode/DeveloperModeWidgetTableMode.h"
#include "KLModelDefinitionCore.h"
#include "PublicDefine.h"

#include <QAbstractTableModel>
#include <QComboBox>
#include <QLineEdit>
#include <QSortFilterProxyModel>
#include <QStyledItemDelegate>

/// @brief 委托
class MyDelegate : public QStyledItemDelegate
{
public:
    MyDelegate(QObject* parent = 0) : QStyledItemDelegate(parent) { }

protected:
    QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;

    void setEditorData(QWidget* editor, const QModelIndex& index) const override;

    void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

    void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                              const QModelIndex& index) const override;

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};
#endif;
#include "DeveloperModeWidgetTableDelegate.h"

#include <QPainter>
#include <QSvgRenderer>

QWidget* MyDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (!index.isValid()) {
        return nullptr;
    }
    QString ketname = Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(
            index.sibling(index.row(), TableModel::Key).data().toString());
    switch (index.column()) {
    case TableModel::Value: {
        if (index.data().toString() == "true" || index.data().toString() == "false") {
            QComboBox* box = new QComboBox(parent);
            box->addItems(QStringList() << "true"
                                        << "false");
            return box;
        } else {
            QLineEdit* editor = new QLineEdit(parent);
            return editor;
        }
    } break;
    default: {
        QLineEdit* editor = new QLineEdit(parent);
        return editor;
    } break;
    }
}

void MyDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QString keyname = Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(
            index.sibling(index.row(), TableModel::Key).data().toString());
    switch (index.column()) {
    case TableModel::Value: {
        bool value = index.data(Qt::DisplayRole).toBool();
        QComboBox* combox = qobject_cast<QComboBox*>(editor);
        if (combox) {
            int curIndex = combox->findText(value ? "true" : "false");
            combox->setCurrentIndex(curIndex);
        } else {
            QLineEdit* edit = qobject_cast<QLineEdit*>(editor);
            if (edit) {
                edit->setText(index.data(Qt::DisplayRole).toString());
            }
        }
    } break;
    default: {
        QLineEdit* edit = qobject_cast<QLineEdit*>(editor);
        if (edit) {
            /// 使用委托的目的就是为了点击单元格时，单元格内的数据不会被清空
            edit->setText(index.data(Qt::DisplayRole).toString());
        }
    } break;
    }
}

void MyDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    QString keyname = Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(
            index.sibling(index.row(), TableModel::Key).data().toString());
    switch (index.column()) {
    case TableModel::Value: {
        QComboBox* combox = qobject_cast<QComboBox*>(editor);
        if (combox) {
            model->setData(index, combox->currentText(), Qt::EditRole);
        } else {
            QLineEdit* edit = qobject_cast<QLineEdit*>(editor);
            if (edit) {
                model->setData(index, edit->text(), Qt::EditRole);
            }
        }
    } break;
    default: {
        QLineEdit* edit = qobject_cast<QLineEdit*>(editor);
        if (edit) {
            model->setData(index, edit->text(), Qt::EditRole);
        }
    } break;
    }
}

void MyDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                                      const QModelIndex& index) const
{
    editor->setGeometry(option.rect);
}

void MyDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{

    // if (index.data(Qt::DecorationRole).type() == QMetaType::QPixmap) {
    //     QPixmap pixmap = qvariant_cast<QPixmap>(index.data(Qt::DecorationRole));
    //     pixmap =
    //             pixmap.scaled(option.rect.size().height(), option.rect.size().height(),
    //             Qt::KeepAspectRatioByExpanding);
    //     painter->drawPixmap(
    //             QRect(option.rect.x(), option.rect.y(), option.rect.size().height(), option.rect.size().height()),
    //             pixmap);
    // } else {
    QStyledItemDelegate::paint(painter, option, index);
    // }
}

#include "controlDialogDelegate.h"
#include "controlDialogModel.h"

#include <QPainter>
#include <QStyleOptionViewItem>
#include <QLineEdit>

ControlDialogDelegate::ControlDialogDelegate(QObject *parent) : QStyledItemDelegate(parent)
{
}

QSize ControlDialogDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    return QStyledItemDelegate::sizeHint(option, index);
}

void ControlDialogDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QStyledItemDelegate::paint(painter, option, index);
}

QWidget *ControlDialogDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    if (index.column() == 1)
    {
        auto *editor = new QLineEdit(parent);
        return editor;
    }
    return nullptr;
}

void ControlDialogDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    if (index.column() == 1)
    {
        auto *lineEdit = qobject_cast<QLineEdit *>(editor);
        lineEdit->setText(index.data().toString());
    }
}

void ControlDialogDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    if (index.column() == 1)
    {
        auto *lineEdit = qobject_cast<QLineEdit *>(editor);
        auto myModel = dynamic_cast<TreeModel *>(model);
        if (myModel)
        {
            auto strs = myModel->getStringSet();
            if (strs && strs->contains(lineEdit->text()))
            {
                model->setData(index, index.data().toString());
            }
            else
            {
                model->setData(index, lineEdit->text());
                myModel->addString(lineEdit->text());
            }
        }
    }
}

void ControlDialogDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}

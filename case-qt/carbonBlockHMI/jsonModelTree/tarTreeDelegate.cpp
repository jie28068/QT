#include "tarTreeDelegate.h"
#include "carTreeView.h"
#include "QJsonModel.h"

#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>

#include <QDebug>
TarTreeDelegate::TarTreeDelegate(QObject *parent) : QStyledItemDelegate(parent)
{
}

int countDecimalPlaces(double value)
{
    QString data = QString::number(value);
    QStringList parts = data.split('.');
    if (parts.length() > 1)
    {
        return parts[1].length();
    }
    return 2;
}

QWidget *TarTreeDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    Q_UNUSED(option)
    if (!index.isValid())
        return nullptr;

    switch (index.column())
    {
    case 0:
    {
        QLineEdit *editor = new QLineEdit(parent);
        editor->setFrame(false);
        return editor;
    }
    case 1:
    {
        auto vData = index.data(Qt::DisplayRole);
        if (vData.type() == QVariant::Bool)
        {
            QComboBox *editor = new QComboBox(parent);
            editor->setFrame(false);
            editor->addItems({"True", "False"});
            return editor;
        }
        else if (vData.type() == QVariant::Int)
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setFrame(false);
            editor->setMinimum(-10000);
            editor->setMaximum(10000);
            return editor;
        }
        else if (vData.type() == QVariant::Double)
        {
            QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
            editor->setFrame(false);
            editor->setMinimum(-10000);
            editor->setMaximum(10000);
            int count = countDecimalPlaces(index.data(Qt::DisplayRole).toDouble());
            editor->setDecimals(count);
            return editor;
        }
        else if (vData.type() == QVariant::String)
        {
            QLineEdit *editor = new QLineEdit(parent);
            editor->setFrame(false);
            return editor;
        }
        break;
    }
    default:
    {
        QLineEdit *editor = new QLineEdit(parent);
        editor->setFrame(false);
        return editor;
    }
    }
    return nullptr;
}

void TarTreeDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    switch (index.column())
    {
    case 1:
    {
        auto vData = index.data(Qt::DisplayRole);
        if (vData.type() == QVariant::Bool)
        {
            QComboBox *box = static_cast<QComboBox *>(editor);
            box->setCurrentIndex(index.data(Qt::DisplayRole).toBool() ? 0 : 1);
        }
        else if (vData.type() == QVariant::Int)
        {
            QSpinBox *spinBox = static_cast<QSpinBox *>(editor);
            spinBox->setValue(index.data(Qt::DisplayRole).toInt());
        }
        else if (vData.type() == QVariant::Double)
        {
            QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(editor);
            spinBox->setValue(index.data(Qt::DisplayRole).toDouble());
        }
        else if (vData.type() == QVariant::String)
        {
            QLineEdit *edit = static_cast<QLineEdit *>(editor);
            edit->setText(index.data(Qt::DisplayRole).toString());
        }
        break;
    }
    default:
    {
        QLineEdit *edit = static_cast<QLineEdit *>(editor);
        edit->setText(index.data(Qt::DisplayRole).toString());
    }
    }
}

void TarTreeDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    auto keyName = index.sibling(index.row(), 0).data().toString();
    switch (index.column())
    {
    case 1:
    {
        auto vData = index.data(Qt::DisplayRole);
        if (vData.type() == QVariant::Bool)
        {
            QComboBox *box = static_cast<QComboBox *>(editor);
            model->setData(index, box->currentIndex() == 0 ? true : false, Qt::EditRole);
        }
        else if (vData.type() == QVariant::Int)
        {
            QSpinBox *spinBox = static_cast<QSpinBox *>(editor);
            model->setData(index, spinBox->value(), Qt::EditRole);
        }
        else if (vData.type() == QVariant::Double)
        {
            QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(editor);
            model->setData(index, spinBox->value(), Qt::EditRole);
        }
        else if (vData.type() == QVariant::String)
        {
            QLineEdit *edit = static_cast<QLineEdit *>(editor);
            // test
            if (keyName == "name") // 名称检测
            {
                if (edit->text() == "nihao123")
                {
                    qDebug() << "不予写入";
                    return;
                }
            }
            model->setData(index, edit->text(), Qt::EditRole);
        }
        break;
    }
    default:
    {
        QLineEdit *edit = static_cast<QLineEdit *>(editor);
        model->setData(index, edit->text(), Qt::EditRole);
    }
    }
}

void TarTreeDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    Q_UNUSED(index)
    editor->setGeometry(option.rect);
}

TreeViewProxyModel::TreeViewProxyModel(QObject *parent) : QSortFilterProxyModel(parent)
{
    m_treeView = qobject_cast<CarTreeView *>(parent);
}

void TreeViewProxyModel::setFilterString(const QString &strFilter)
{
    m_strFilterString = strFilter;
    invalidateFilter();
    // 遍历筛选后的节点并展开
    expandFilteredNodes();
}

void TreeViewProxyModel::expandFilteredNodes(const QModelIndex &parent)
{
    if (!m_treeView)
    {
        return;
    }
    int rowCount = m_treeView->model()->rowCount(parent);
    for (int i = 0; i < rowCount; ++i)
    {
        QModelIndex child = m_treeView->model()->index(i, 0, parent);
        // 仅对当前没有展开的节点进行处理
        if (m_treeView->model()->hasChildren(child) && !m_treeView->isExpanded(child))
        {
            m_treeView->expand(child);
        }
        // 递归
        expandFilteredNodes(child);
    }
}

bool TreeViewProxyModel::filterAcceptsRow(int source_row, const QModelIndex &source_parent) const
{
    QModelIndex index = sourceModel()->index(source_row, 0, source_parent);
    QString strDisplayName = index.data(Qt::DisplayRole).toString();

    bool m_pFilter = strDisplayName.contains(m_strFilterString, Qt::CaseInsensitive);
    if (m_pFilter)
    {
        return true;
    }
    else
    {
        // 不符合条件的父节点 对其子节点进行递归判断处理
        for (int i = 0; i < sourceModel()->rowCount(index); i++)
        {
            if (TreeViewProxyModel::filterAcceptsRow(i, index))
            {
                return true;
            }
        }
        return false;
    }
}

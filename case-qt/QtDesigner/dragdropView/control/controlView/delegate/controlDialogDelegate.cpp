#include "controlDialogDelegate.h"

#include <QPainter>
#include <QStyleOptionViewItem>

ControlDialogDelegate::ControlDialogDelegate(QObject *parent) : QStyledItemDelegate(parent)
{
}

void ControlDialogDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    // 绘制默认样式
    QStyledItemDelegate::paint(painter, option, index);

    // 绘制边框
    painter->save();
    QRect rect = option.rect;
    QPen pen(Qt::black, 1, Qt::SolidLine);
    painter->setPen(pen);

    // 第一列边框大小与上级一致
    if (index.column() == 0 && index.row() == 0)
    {
        // m_rect = option.rect;
        painter->drawRect(rect);
    }
    else
    {
        // rect.setWidth(m_rect.width());
        // rect.setHeight(m_rect.height());

        // 根据需要调整其他列的边框大小
        painter->drawRect(rect.adjusted(0, 0, -1, -1));
    }

    painter->restore();

    // // 绘制展开/折叠指示器
    // if (index.model()->hasChildren(index))
    // {
    //     QRect indicatorRect = option.rect;
    //     // 根据需要调整指示器的位置和大小
    //     indicatorRect.setWidth(20);
    //     painter->drawPixmap(indicatorRect, option.widget->style()->standardPixmap(QStyle::SP_ArrowRight));
    // }
}

QSize ControlDialogDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    return QStyledItemDelegate::sizeHint(option, index);
}

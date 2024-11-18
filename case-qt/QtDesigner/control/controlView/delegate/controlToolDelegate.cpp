#include "controlToolDelegate.h"
#include "globalDefinition.h"

#include <QPainter>
ControlDelegate::ControlDelegate(QObject *parent) : QStyledItemDelegate(parent)
{
}

void ControlDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QPixmap image = index.data(GlobalDefinition::controlModelPixmap).value<QPixmap>();
    if (!image.isNull())
    {
        painter->drawPixmap(option.rect, image);
    }
    else
    {
        painter->drawText(option.rect, Qt::AlignCenter, index.data(GlobalDefinition::controlModelName).toString());
    }
    painter->restore();
}

QSize ControlDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    // return QStyledItemDelegate::sizeHint(option, index);
    return QSize(GlobalDefinition::imageWidth, GlobalDefinition::imageHeight);
}

// void ControlDelegate::drawFrame(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
// {
//     QPen pen;
//     pen.setWidth(1);
//     pen.setColor(Qt::black);
//     painter->setPen(pen);
//     painter->drawRect(option.rect.adjusted(0, 0, -1, -1));
// }

// void ControlDelegate::drawText(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
// {
//     QPen pen;
//     pen.setWidth(1);
//     pen.setColor(Qt::black);
//     painter->setPen(pen);
//     QString text = index.data().toString();
//     painter->drawText(option.rect, Qt::AlignCenter, text);
// }

// void ControlDelegate::drawBackground(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
// {
// }

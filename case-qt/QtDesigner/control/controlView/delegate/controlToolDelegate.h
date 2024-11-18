#ifndef CONTROLTOOLDELEGATE_H
#define CONTROLTOOLDELEGATE_H

#include <QStyledItemDelegate>
class ControlDelegate : public QStyledItemDelegate
{
public:
    ControlDelegate(QObject *parent = 0);
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;

    // void drawFrame(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    // void drawText(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    // void drawBackground(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
};
#endif
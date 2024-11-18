#ifndef CONTROLDIALOGDELEGATE_H
#define CONTROLDIALOGDELEGATE_H

#include <QStyledItemDelegate>

class ControlDialogDelegate : public QStyledItemDelegate
{
public:
    ControlDialogDelegate(QObject *parent = 0);
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;

private:
    mutable QRect m_rect;
};
#endif
#ifndef CONTROLTOOLISTVIEW_H
#define CONTROLTOOLISTVIEW_H

#include <QListView>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QRect>
#include <QApplication>

class CustomDelegate : public QStyledItemDelegate
{
public:
    CustomDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;
};

class ControlToolListView : public QListView
{
public:
    ControlToolListView(QString text, QWidget *parent = 0);
    ~ControlToolListView();

private:
    QString m_text;
};

#endif
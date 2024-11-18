#ifndef CONTROLFLOATTREEWIDGET_H
#define CONTROLFLOATTREEWIDGET_H

#include <QWidget>
#include <QTreeView>
#include "globalDefinition.h"

class ControlFloatTreeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ControlFloatTreeWidget(QWidget *parent = nullptr);
    ~ControlFloatTreeWidget();

private:
    QVector<GlobalDefinition::Province *> initData();
    void setModel(const QVector<GlobalDefinition::Province *> &proList);

signals:
    void pClicked(bool checked = false);

private slots:
    void onTreeDataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight);

private:
    QTreeView *treeView;
};
#endif
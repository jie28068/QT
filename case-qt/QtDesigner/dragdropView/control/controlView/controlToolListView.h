#ifndef CONTROLTOOLISTVIEW_H
#define CONTROLTOOLISTVIEW_H

#include <QListView>

class ControlToolListView : public QListView
{
public:
    ControlToolListView(QString text, QWidget *parent = 0);
    ~ControlToolListView();

protected:
    bool viewportEvent(QEvent *event) override;

private:
    QString m_text;
};

#endif
#ifndef CONTROLTOOLISTVIEW_H
#define CONTROLTOOLISTVIEW_H

#include <QListView>

class ControlToolListView : public QListView
{
    Q_OBJECT
public:
    ControlToolListView(QString text, QWidget *parent = 0);
    ~ControlToolListView();

protected:
    bool viewportEvent(QEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

signals:
    void clickedControl(QMimeData *mimeData);

private:
    QString m_text;
};

#endif
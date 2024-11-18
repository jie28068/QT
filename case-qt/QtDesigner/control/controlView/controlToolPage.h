#ifndef CONTROLTOOLPAGE_H
#define CONTROLTOOLPAGE_H

#include <QMimeData>
#include <QWidget>

class ControlToolButtons;
class ControlToolListView;
class ControlToolPage : public QWidget
{
    Q_OBJECT
public:
    ControlToolPage(QString text, QWidget *parent = 0);
    ~ControlToolPage();

signals:
    void clickedControlPage(QMimeData *data);

private:
    QString m_text;
    ControlToolButtons *m_button;
    ControlToolListView *m_listView;
};

#endif
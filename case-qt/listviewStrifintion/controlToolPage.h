#ifndef CONTROLTOOLPAGE_H
#define CONTROLTOOLPAGE_H

#include <QWidget>

class ControlToolButtons;
class ControlToolListView;
class ControlToolPage : public QWidget
{
public:
    ControlToolPage(QString text, QWidget *parent = 0);
    ~ControlToolPage();

private:
    QString m_text;
    ControlToolButtons *m_button;
    ControlToolListView *m_listView;
};

#endif
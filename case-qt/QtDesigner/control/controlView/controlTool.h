#ifndef CONTROLTOOL_H
#define CONTROLTOOL_H

#include <QWidget>
#include <QVBoxLayout>
#include <QMimeData>

class ControlToolPage;
class ControlTools : public QWidget
{
    Q_OBJECT
public:
    explicit ControlTools(QWidget *parent = 0);
    ~ControlTools();

    ControlToolPage *addPage(QString title);
signals:
    void clickedControlTools(QMimeData *mimeData);

private:
    QVBoxLayout *m_contentLayout;
};

#endif
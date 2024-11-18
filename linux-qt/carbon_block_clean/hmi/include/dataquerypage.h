#ifndef DATAQUERYPAGE_H
#define DATAQUERYPAGE_H

#include <QWidget>

namespace Ui
{
    class DataQueryPage;
}

class DataQueryPage : public QWidget
{
    Q_OBJECT

public:
    explicit DataQueryPage(QWidget *parent = nullptr);
    ~DataQueryPage();

private:
    Ui::DataQueryPage *ui;
};

#endif // DATAQUERYPAGE_H

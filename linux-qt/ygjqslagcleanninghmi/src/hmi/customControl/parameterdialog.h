#pragma once

#include <QDialog>

namespace Ui
{
    class ParameterDialog;
}

class QtNode;
class ParameterDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ParameterDialog(QtNode *node_ptr, QWidget *parent = nullptr);
    ~ParameterDialog();

private:
    Ui::ParameterDialog *ui;
    QtNode *node_ptr;

    bool line = false;
    bool lineX = false;
    bool lineY = false;
    bool lineZ = false;
};

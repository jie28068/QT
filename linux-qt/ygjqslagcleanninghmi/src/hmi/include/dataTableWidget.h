#ifndef DATATABLEWIDGET_H
#define DATATABLEWIDGET_H

#include <QTableWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QHeaderView>
#include <QMessageBox>
#include <QLabel>

class QtNode;
class DataTableWidget : public QWidget
{
    Q_OBJECT
public:
    DataTableWidget(QtNode *node_ptr, QString str, QWidget *parent = nullptr);

    void loadData();

    void displayPage(int page);

private slots:
    void onPreviousPage();

    void onNextPage();

    void onRefresh();

    void onPreviousRowPage();

    void onNextRowPage();

private:
    void setupUI();
    void connectSignalsSlots();
    void updatePageLabel();
    /// 更新数据
    void updateRowPageLabel();

private:
    QTableWidget *tableWidget;
    QPushButton *previousButton;
    QPushButton *nextButton;
    QPushButton *previousPageButton;
    QPushButton *nextPageButton;
    QPushButton *refreshButton;
    QLabel *pageLabel;
    QLabel *pageSizeLabel;
    QLabel *allDataLabel;
    int pageSize = 10;   // 每页显示的行数
    int totalRows = 0;   // 总行数
    int totalPage = 0;   // 总页数
    int currentPage = 1; // 当前页码
    QtNode *qt_node_;
    QVector<QVector<QString>> dataSource; // 数据源
    QString logType;
};

#endif
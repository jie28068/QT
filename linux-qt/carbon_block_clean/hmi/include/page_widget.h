#ifndef PAGE_WIDGET_H
#define PAGE_WIDGET_H

#include <QWidget>
#include <QList>
#include <QLabel>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>
#include <QtGlobal>
#include <QtMath>
#include <QKeyEvent>

namespace Ui {
class Page_widget;
}

class Page_widget : public QWidget
{
    Q_OBJECT

public:
    // 翻页显示分成三个部分，左...中...右，blockSize表示每部分的标签个数
    explicit Page_widget(int blockSize = 3, QWidget *parent = 0);
    ~Page_widget();

    int getBlockSize() const;   // 获取每部分的标签个数
    void setBlockSize(int blockSize);    // 设置每部分的标签个数 每部分的标签个数， block size必须是奇数，且最小为3
    int getMaxPage() const;     // 获取总页数
    void setMaxPage(int maxPage);   // 设置总页数 maxPage 总页数值
    int getCurrentPage() const;     // 获取当前页数
    /**
     * @brief setCurrentPage 设置当前页
     * @param currentPage   当前页数值
     * @param signalEmitted 为true时发送currentPageChanged(int)信号
     */
    void setCurrentPage(int currentPage, bool signalEmitted = false);

protected:
    /**
     * @brief eventFilter   事件过滤器，响应上一页标签和下一页标签的点击事件
     * @param watched       发生事件的组件
     * @param e             发生事件的类型
     * @return
     */
    virtual bool eventFilter(QObject *watched, QEvent *e);

signals:
    /**
     * @brief currentPageChanged 当前页信号
     * @param page               页码
     */
    void currentPageChanged(int page);

private:
    Ui::Page_widget *ui;

    // 字体
    QFont font;
    // 前一页， "<"
    QLabel *previousPageLabel = nullptr;
    // 左侧部分标签的容器
    QWidget *leftPagesWidget = nullptr;
    // 左侧分隔符， ".."
    QLabel *leftSeparateLabel = nullptr;
    // 中间部分标签的容器
    QWidget *centerPagesWidget = nullptr;
    // 右侧分隔符， ".."
    QLabel *rightSeparateLabel = nullptr;
    // 右侧部分标签的容器
    QWidget *rightPagesWidget = nullptr;
    // 下一页，">"
    QLabel *nextPageLabel = nullptr;


    int blockSize;  // 翻页显示分成三个部分，左..中..右，blockSize表示每部分的标签个数
    int maxPage; // 总页数
    int currentPage; // 当前页
    QList<QLabel *> *pageLabels;    // 存储所有的数字标签，总个数为blockSize*3

    void initialize(); // 标签初始化
    void updatePageLabels(); // 更新显示标签
    void initPageWidget(int blockSize);  // 初始化pageWidget
};

#endif // PAGE_WIDGET_H

#ifndef CONTROLDIALOGTREEVIEW_H
#define CONTROLDIALOGTREEVIEW_H

#include <QProxyStyle>
#include <QTreeView>

class ControlDialogTreeView : public QTreeView
{
public:
    explicit ControlDialogTreeView(QWidget *parent = nullptr);

protected:
    bool viewportEvent(QEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    /// @note 拖入鼠标为禁止状态，要实现该函数
    void dragMoveEvent(QDragMoveEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

public:
    void insertRow(QByteArray encodedData, QModelIndex index);
    void insertChild(QByteArray encodedData, QModelIndex index);
    void removeRow();
    QRect getRowVisualRect(const QModelIndex &index);

private:
    QModelIndex m_hoveredIndex; // 当前拖拽的Index
    bool m_isMyselfDrop = true; // 是否为本身的拖拽
    QMenu *contextMenu;
};

/// @brief 自绘指示器
class ControlViewStyle : public QProxyStyle
{
public:
    ControlViewStyle(QStyle *style = 0);

    void drawPrimitive(PrimitiveElement element, const QStyleOption *option, QPainter *painter, const QWidget *widget = 0) const override;
};

#endif
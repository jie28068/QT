#include "controlToolListView.h"

#include <QEvent>

ControlToolListView::ControlToolListView(QString text, QWidget *parent) : QListView(parent), m_text(text)
{
    // 设置焦点策略为点击获取焦点
    setFocusPolicy(Qt::ClickFocus);
    // 设置边框样式为无边框
    setFrameShape(QFrame::NoFrame);
    // 设置所有项的大小一致
    setUniformItemSizes(true);
    // 启用鼠标追踪
    setMouseTracking(true);

    setViewMode(QListView::ListMode);
    // 设置调整模式为自适应
    setResizeMode(QListView::Adjust);

    setFlow(QListView::TopToBottom); // 设置流为垂直
    setWrapping(true);               // 允许项目换行
    setIconSize(QSize(50, 50));
    setSpacing(10);
    setDragEnabled(false);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    setItemDelegate(new CustomDelegate());

    // setStyleSheet("QListView::item:hover { background: transparent; border: none; }");
}

ControlToolListView::~ControlToolListView()
{
}

void CustomDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    int radius = 10;
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(Qt::NoPen);
    if (index.model()->data(index, Qt::UserRole).toBool())
    {
        painter->setBrush(QColor("#d9d9d9"));
        painter->drawRoundedRect(option.rect.adjusted(2, 2, -2, -2), radius, radius);
    }

    if (option.state & QStyle::State_MouseOver || option.state & QStyle::State_Selected)
    {
        painter->setBrush(QColor("#e4e4e4"));
        painter->drawRoundedRect(option.rect.adjusted(2, 2, -2, -2), radius, radius);
    }
    if (QStyle::State_HasFocus & option.state)
    {
    }
    QStyledItemDelegate::paint(painter, option, index);
}

QSize CustomDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QSize size = QStyledItemDelegate::sizeHint(option, index);
    size.setWidth(option.rect.width() / 2);
    return size;
}

bool CustomDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{
    if (event->type() == QEvent::MouseMove || event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonDblClick)
    {
        return true;
    }
    return QStyledItemDelegate::editorEvent(event, model, option, index);
}

#include "controlToolListView.h"
#include "globalDefinition.h"

#include <QDropEvent>
#include <QToolTip>
#include <QEvent>
#include <QMimeData>
/**
 * 构造函数: 创建一个ControlToolListView实例。
 * @param text 列表视图上的文本标签。
 * @param parent 父QWidget对象。
 */
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

    // 设置视图模式为图标模式
    setViewMode(QListView::IconMode);
    // 设置调整模式为自适应
    setResizeMode(QListView::Adjust);
    // 设置流方向为从左到右
    setFlow(QListView::LeftToRight);
    // 设置选择模式为单选
    setSelectionMode(QAbstractItemView::SingleSelection);
    // 设置选择行为为选择item
    setSelectionBehavior(QAbstractItemView::SelectItems);

    setDragEnabled(true);
    setDragDropMode(QAbstractItemView::DragOnly);
}

ControlToolListView::~ControlToolListView()
{
}

bool ControlToolListView::viewportEvent(QEvent *event)
{
    if (event->type() == QEvent::ToolTip)
    {
        QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
        QModelIndex index = indexAt(helpEvent->pos());
        if (index.isValid())
        {
            QString tooltipText = model()->data(index, Qt::ToolTipRole).toString();
            if (!tooltipText.isEmpty())
            {
                QToolTip::showText(helpEvent->globalPos(), tooltipText);
                return true;
            }
        }
    }
    return QListView::viewportEvent(event);
}

void ControlToolListView::mousePressEvent(QMouseEvent *event)
{
    QModelIndex index = indexAt(event->pos());
    QMimeData *mimeData = new QMimeData();
    // 保存数据的形式
    QByteArray encodedData;
    QDataStream stream(&encodedData, QIODevice::WriteOnly);

    if (index.isValid())
    {
        QString name = model()->data(index, GlobalDefinition::controlModelName).toString();
        QPixmap pixmap = qvariant_cast<QPixmap>(model()->data(index, GlobalDefinition::controlModelPixmap));
        QString type = model()->data(index, GlobalDefinition::controlModelType).toString();
        stream << name << pixmap << type;
        mimeData->setData(GlobalDefinition::controlMimeType, encodedData);
        emit clickedControl(mimeData);
    }
}
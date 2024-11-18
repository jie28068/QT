#include "controlDialogTreeView.h"
#include "controlDialogModel.h"
#include "globalDefinition.h"
#include "controlDialogDelegate.h"
#include "ControlOneToMany.h"
#include "ControlManyToOne.h"

#include <QMimeData>
#include <QPainter>
#include <QFile>
#include <QEvent>
#include <QHelpEvent>
#include <QToolTip>
#include <QMenu>
#include <QDataStream>
#include <QHeaderView>
ControlDialogTreeView::ControlDialogTreeView(QWidget *parent) : QTreeView(parent)
{
    setDragEnabled(false);
    setAcceptDrops(false);
    setDropIndicatorShown(false);
    viewport()->setAcceptDrops(false);
    setDragDropMode(QAbstractItemView::NoDragDrop);
    setStyle(new ControlViewStyle(style()));
    setItemDelegate(new ControlDialogDelegate(this));

    const QStringList headers({"类型", "名称"});

    TreeModel *treeModel = new TreeModel(headers);
    this->setModel(treeModel);
    for (int column = 0; column < treeModel->columnCount(); ++column)
    {
        this->resizeColumnToContents(column);
    }

    // qss
    QFile files("D:\\GitProject\\my-case\\QT\\QtDesigner\\Recourse\\style.qss");
    if (files.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        setStyleSheet(files.readAll());
        files.close();
    }

    // menu
    contextMenu = new QMenu(this);
    QAction *action = new QAction("删除", this);
    contextMenu->addAction(action);
    connect(action, &QAction::triggered, this, &ControlDialogTreeView::removeRow);
}

bool ControlDialogTreeView::viewportEvent(QEvent *event)
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
            }
        }
    }
    return QTreeView::viewportEvent(event);
}

void ControlDialogTreeView::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasFormat(GlobalDefinition::controlMimeType) || event->mimeData()->hasFormat(GlobalDefinition::controlDialogMimeType))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void ControlDialogTreeView::dropEvent(QDropEvent *event)
{
    QTreeView::dropEvent(event);
    // 检查拖放事件的目标位置
    QModelIndex index = indexAt(event->pos());
    // 根据需要设置拖放指示器的状态
    if (index.isValid())
    {
        // 在有效的索引位置设置拖放指示器
        setDropIndicatorShown(true);
    }
    else
    {
        // 在无效的索引位置隐藏拖放指示器
        setDropIndicatorShown(false);
    }
    auto data = event->mimeData();
    QByteArray encodedData;
    if (data->hasFormat(GlobalDefinition::controlMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlMimeType);
    }
    else if (data->hasFormat(GlobalDefinition::controlDialogMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlDialogMimeType);
    }
    DropIndicatorPosition dropindicationPos = dropIndicatorPosition();
    switch (dropindicationPos)
    {
    case QAbstractItemView::AboveItem:
    {
        insertRow(encodedData, index);
    }
    break;
    case QAbstractItemView::BelowItem:
    {
        insertRow(encodedData, index);
    }
    break;
    case QAbstractItemView::OnItem:
    {
        insertChild(encodedData, index);
    }
    break;
    case QAbstractItemView::OnViewport:
    {
        insertChild(encodedData, index);
    }
    break;
    }
    event->setDropAction(Qt::MoveAction); /// @note 设置该次修改的动作
    event->accept();

    m_isMyselfDrop = true;
}

void ControlDialogTreeView::dragMoveEvent(QDragMoveEvent *event)
{

    QTreeView::dragMoveEvent(event);
    auto data = event->mimeData();
    if (!data->hasFormat(GlobalDefinition::controlDialogMimeType))
    {
        m_isMyselfDrop = false;
    }
    QModelIndex index = indexAt(event->pos());
    if (index.isValid())
    {
        m_hoveredIndex = index;
        update();
    }
    else
    {
        m_hoveredIndex = QModelIndex();
    }
    event->accept();
}

void ControlDialogTreeView::mouseMoveEvent(QMouseEvent *event)
{
    QTreeView::mouseMoveEvent(event);
}

QRect ControlDialogTreeView::getRowVisualRect(const QModelIndex &index)
{
    if (!index.isValid())
        return QRect();

    // 获取模型的列数
    int columnCount = this->model()->columnCount(index.parent());

    QRect rowRect;

    // 遍历所有列，获取索引指向行上每个项的视觉矩形，然后将它们合并
    for (int column = 0; column < columnCount; ++column)
    {
        QModelIndex columnIndex = this->model()->index(index.row(), column, index.parent());
        QRect rect = this->visualRect(columnIndex);

        if (column == 0)
            rowRect = rect;
        else
            rowRect = rowRect.united(rect);
    }

    return rowRect;
}

void ControlDialogTreeView::paintEvent(QPaintEvent *event)
{
    QTreeView::paintEvent(event);

    if (m_hoveredIndex.isValid() && !m_isMyselfDrop)
    {
        QPainter painter(viewport());
        QRect rect = visualRect(m_hoveredIndex);
        QRect rowRect = getRowVisualRect(m_hoveredIndex);
        QPen pen;
        pen.setColor(QColor(qRgb(135, 206, 235)));
        pen.setWidth(2);
        painter.setPen(pen);
        painter.setRenderHint(QPainter::Antialiasing);

        DropIndicatorPosition dropindicationPos = dropIndicatorPosition();
        switch (dropindicationPos)
        {
        case QAbstractItemView::AboveItem:
        {
            painter.drawEllipse(QPoint(10, rowRect.top()), 4, 4);
            painter.drawLine(QPoint(10, rect.top()), QPoint(rowRect.width() + 10, rect.top()));
        }
        break;
        case QAbstractItemView::BelowItem:
        {
            painter.drawEllipse(QPoint(10, rowRect.top()), 4, 4);
            painter.drawLine(QPoint(10, rect.top()), QPoint(rowRect.width() + 10, rect.top()));
        }
        break;
        case QAbstractItemView::OnItem:
        {
            rect.setLeft(5);
            rect.setRight(rowRect.width() + 5);
            painter.drawRect(rect);
        }
        break;
        case QAbstractItemView::OnViewport:
            break;
        }
    }
}

void ControlDialogTreeView::contextMenuEvent(QContextMenuEvent *event)
{
    contextMenu->exec(event->globalPos());
}

void ControlDialogTreeView::insertRow(QByteArray encodedData, QModelIndex index)
{
    QAbstractItemModel *model = this->model();

    QDataStream stream(&encodedData, QIODevice::ReadOnly);
    while (!stream.atEnd())
    {
        QString name;
        QString pixmap;
        QString type;
        stream >> name >> pixmap >> type;
        model->insertRow(index.row(), index.parent());
        for (int column = 0; column < model->columnCount(index.parent()); ++column)
        {
            const QModelIndex child = model->index(index.row(), column, index.parent());
            if (column == 0)
            {
                model->setData(child, name, Qt::EditRole);
            }
            else if (column == 1)
            {
                model->setData(child, type, Qt::EditRole);
            }
        }
    }
}

void ControlDialogTreeView::insertChild(QByteArray encodedData, QModelIndex index)
{
    QAbstractItemModel *model = this->model();
    int row = 0;
    if (index.row() < 0)
    {
        row = model->rowCount();
    }
    if (!model->insertRow(row, index))
        return;

    QDataStream stream(&encodedData, QIODevice::ReadOnly);
    while (!stream.atEnd())
    {
        QString name;
        QString pixmap;
        QString type;
        stream >> name >> pixmap >> type;
        ControlBase *edit = nullptr;
        if (name == GlobalDefinition::controlOneToMany)
        {
            edit = new ControlOneToMany();
        }
        else if (name == GlobalDefinition::controlManyToOne)
        {
            edit = new ControlManyToOne();
        }
        int i = 1;
        while (i > 0)
        {
            QString name = QString("Parameters%1").arg(i++);
            if (dynamic_cast<TreeModel *>(this->model())->addString(name))
            {
                edit->setName(name);
                break;
            }
        }
        const QModelIndex child = model->index(row, 0, index);
        const QModelIndex child2 = model->index(row, 1, index);
        QVariant variant = QVariant::fromValue(edit);
        model->setData(child, variant, Qt::EditRole);
        model->setData(child2, edit->getName(), Qt::EditRole);
    }

    this->selectionModel()->setCurrentIndex(model->index(0, 0, index),
                                            QItemSelectionModel::ClearAndSelect);
}

void ControlDialogTreeView::removeRow()
{
    const QModelIndex index = this->selectionModel()->currentIndex();
    QAbstractItemModel *model = this->model();
    auto name = index.siblingAtColumn(1).data().toString();
    if (model->removeRow(index.row(), index.parent()))
    {
        auto stringSet = dynamic_cast<TreeModel *>(model)->getStringSet();
        if (stringSet->contains(name))
        {
            stringSet->remove(name);
        }
    }
}

void ControlDialogTreeView::resizeEvent(QResizeEvent *event)
{
    QTreeView::resizeEvent(event);
    if (header()->count() > 1)
    {
        int totalWidth = viewport()->width();
        int widthForFirstColumn = static_cast<int>(totalWidth * 0.3);
        setColumnWidth(0, widthForFirstColumn);
    }
}

ControlViewStyle::ControlViewStyle(QStyle *style)
    : QProxyStyle(style)
{
}

void ControlViewStyle::drawPrimitive(PrimitiveElement element, const QStyleOption *option, QPainter *painter, const QWidget *widget) const
{
    if (element == QStyle::PE_IndicatorItemViewItemDrop && !option->rect.isNull())
    {
        QRect rect = option->rect;
        QPen pen;
        pen.setColor(QColor(qRgb(135, 206, 235)));
        pen.setWidth(2);
        painter->setPen(pen);
        painter->setRenderHint(QPainter::Antialiasing);
        if (option->rect.height() == 0)
        {
            painter->drawEllipse(QPoint(10, option->rect.top()), 4, 4);
            painter->drawLine(QPoint(10, rect.top()), QPoint(widget->width() - 10, rect.top()));
        }
        else
        {
            rect.setLeft(5);
            rect.setRight(widget->width() - 5);
            painter->drawRect(rect);
        }

        return;
    }
    QProxyStyle::drawPrimitive(element, option, painter, widget);
}
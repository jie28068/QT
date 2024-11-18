#include "puzzlewidget.h"

PuzzleWidget::PuzzleWidget(int size, QWidget *parent) : QWidget(parent), m_ImageSize(size)
{
    setAcceptDrops(true);
    setMinimumSize(m_ImageSize, m_ImageSize);
    setMaximumSize(m_ImageSize, m_ImageSize);
}

int PuzzleWidget::pieceSize() const
{
    return m_ImageSize / MacroDf::getCloum();
}

int PuzzleWidget::imageSize() const
{
    return m_ImageSize;
}

void PuzzleWidget::clear()
{
    pieces.clear();
    highlightedRect = QRect();
    inPlace = 0;
    update();
}

void PuzzleWidget::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasFormat("DJ-NB"))
        event->accept();
    else
        event->ignore();
}

void PuzzleWidget::dragLeaveEvent(QDragLeaveEvent *event)
{
    // 当前高亮区域临时保存
    QRect updateRect = highlightedRect;
    // 原有高亮设置为空，表示没有高亮
    highlightedRect = QRect();
    // 更新高亮区域
    update(updateRect);
    // 接受拖拽事件。这告诉系统已经处理了这个事件，不需要进一步传递给其他部件
    event->accept();
}

void PuzzleWidget::dragMoveEvent(QDragMoveEvent *event)
{
    // 计算高亮区域与目标位置区域的并集,刷新高亮区域
    QRect updateRect = highlightedRect.united(targetSquareMove(event->pos()));
    // 目标位置是否为空，不为空不给放置
    if (event->mimeData()->hasFormat("DJ-NB"))
    {
        bool isvaild = dragMoveIsValid(event->pos());
        if (isvaild)
        {
            event->ignore();
            return;
        }
        highlightedRect = targetSquareMove(event->pos());
        if (findPiece(highlightedRect) == -1)
        {
            event->setDropAction(Qt::MoveAction);
            event->accept();
        }
        else
        {
            if (event->source() == this)
            {
                event->setDropAction(Qt::MoveAction);
                event->accept();
            }
            else
            {
                // todo 暂不支持list与widget交换图片
                event->ignore();
            }
        }
    }
    else
    {
        highlightedRect = QRect();
        event->ignore();
    }

    update(updateRect);
}

void PuzzleWidget::dropEvent(QDropEvent *event)
{
    // 检查事件是否含有我们需要的数据格式
    if (event->mimeData()->hasFormat("DJ-NB"))
    {
        // 接受事件默认的复制动作
        event->setDropAction(Qt::MoveAction);
        event->accept();
        auto square = targetSquareMove(event->pos()); // 目标位置
        int existingPieceIndex = findPiece(square);   // 寻找目标位置是否有拼图块

        // 从拖放事件的数据中读取拼图块的信息
        QByteArray pieceData = event->mimeData()->data("DJ-NB");
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        // 将拼图块添加到列表中或与现有拼图块交换
        if (existingPieceIndex == -1)
        {
            // 目标位置没有拼图块，直接放置新拼图块
            Piece piece;
            piece.rect = targetSquareMove(event->pos());
            dataStream >> piece.pixmap >> piece.location;
            // 将拼图块添加到列表中
            pieces.append(piece);
            // 清除高亮的区域并更新拼图块的区域
            highlightedRect = QRect();
            update(piece.rect);
            // 如果拼图块放置在正确的位置
            addInPlace(piece);
        }
        else
        {
            // 目标位置已有拼图块，和拖入的拼图块互换位置
            // 起始位置资源
            Piece piece;
            dataStream >> piece.pixmap >> piece.location >> piece.rect;
            // 目标位置资源
            Piece rPic = pieces[existingPieceIndex];
            // 删除掉原有的，以便重新写入新值
            if (rPic.location == rPic.rect.topLeft() / pieceSize())
                inPlace--;
            pieces.takeAt(existingPieceIndex);
            // 数据交互
            Piece tempPiece = piece;
            piece.location = rPic.location;
            piece.pixmap = rPic.pixmap;
            rPic.location = tempPiece.location;
            rPic.pixmap = tempPiece.pixmap;
            // 存放俩组数据
            pieces.append(piece);
            pieces.append(rPic);
            // 重绘涉及的区域
            highlightedRect = QRect();
            update(piece.rect);
            update(rPic.rect);
            // 如果拼图块放置在正确的位piece
            addInPlace(rPic);
            addInPlace(piece);
        }
    }
    else
    {
        highlightedRect = QRect();
        // 不是我们支持的数据格式，保留默认行为
        event->ignore();
    }
}

void PuzzleWidget::mousePressEvent(QMouseEvent *event)
{
    // 获取鼠标点击位置的方块
    QRect square = targetSquareMove(event->pos());
    // 查找方块是否有图片
    int found = findPiece(square);

    if (found == -1)
        return;
    // 移除找到的拼图块
    Piece piece = pieces.takeAt(found);
    // 如果拼图块的位置与方块的顶点位置一致，表示该拼图块为正确位置，移除时更新完成计数位
    if (piece.location == square.topLeft() / pieceSize())
        inPlace--;

    update(square);
    // 将拼图块的图像和位置信息存入数据流
    QByteArray itemData;
    QDataStream dataStream(&itemData, QIODevice::WriteOnly);

    dataStream << piece.pixmap << piece.location << piece.rect;
    // 创建拖动操作的数据对象
    QMimeData *mimeData = new QMimeData;
    mimeData->setData("DJ-NB", itemData);
    // 创建拖动操作
    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setHotSpot(event->pos() - square.topLeft());
    drag->setPixmap(piece.pixmap);
    // 判断拖动操作的结果是否为Qt::IgnoreAction，表示拖拽失败，将拼图块放回原位置
    if (drag->exec(Qt::MoveAction) == Qt::IgnoreAction) // 拖放到其他应用程序。我们使用Qt::IgnoreAction来限制它。
    {
        pieces.insert(found, piece);
        update(targetSquareMove(event->pos()));

        if (piece.location == QPoint(square.x() / pieceSize(), square.y() / pieceSize()))
            inPlace++;
    }
}

void PuzzleWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(event->rect(), Qt::white);

    if (highlightedRect.isValid())
    {
        painter.setBrush(QColor("#98FB98"));
        painter.setPen(Qt::NoPen);
        painter.drawRect(highlightedRect.adjusted(0, 0, -1, -1));
    }

    for (const Piece &piece : pieces)
    {
        painter.drawPixmap(piece.rect, piece.pixmap);
    }
}

int PuzzleWidget::findPiece(const QRect &pieceRect) const
{
    for (int i = 0, size = pieces.size(); i < size; ++i)
    {
        if (pieces.at(i).rect == pieceRect)
            return i;
    }
    return -1;
}

const QRect PuzzleWidget::targetSquareMove(const QPoint &position) const
{
    // point除以一个数是往前进位，这会导致坐标出现问题，所以要用Int处理
    int x = position.x() / pieceSize();
    int y = position.y() / pieceSize();
    auto pointNew = QPoint(x, y);
    auto point = pointNew * pieceSize();
    auto resultRect = QRect(point.x(), point.y(), pieceSize(), pieceSize());
    return resultRect;
}

void PuzzleWidget::addInPlace(Piece piece)
{
    if (piece.location == piece.rect.topLeft() / pieceSize())
    {
        inPlace++;
        if (inPlace == MacroDf::getCloum() * MacroDf::getCloum())
            emit puzzleCompleted();
    }
}

bool PuzzleWidget::dragMoveIsValid(const QPoint &position)
{
    // int x = position.x() / pieceSize();
    // int y = position.y() / pieceSize();
    // auto pointNew = QPoint(x, y);
    // auto point = pointNew * pieceSize();
    // // 是否越界
    // if (point.x() > pieceSize() * 2 || point.y() > pieceSize() * 2)
    // {
    //     return true;
    // }
    return false;
}

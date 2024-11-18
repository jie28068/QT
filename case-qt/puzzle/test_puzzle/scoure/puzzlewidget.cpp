#include "../include/puzzlewidget.h"

PuzzleWidget::PuzzleWidget(int size, QWidget *parent) : QWidget(parent), m_ImageSize(size)
{
    setAcceptDrops(true);
    setMinimumSize(m_ImageSize, m_ImageSize);
    setMaximumSize(m_ImageSize, m_ImageSize);
}

int PuzzleWidget::pieceSize() const
{
    return m_ImageSize / 4;
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
    if (event->mimeData()->hasFormat("image/x-puzzle-piece"))
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
    // 计算高亮区域与目标位置区域的并集
    QRect updateRect = highlightedRect.united(targetSquare(event->pos()));
    // 目标位置是否为空，不为空不给放置
    if (event->mimeData()->hasFormat("image/x-puzzle-piece") && findPiece(targetSquare(event->pos())) == -1)
    {
        highlightedRect = targetSquare(event->pos());
        event->setDropAction(Qt::MoveAction);
        event->accept();
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
    if (event->mimeData()->hasFormat("image/x-puzzle-piece") && findPiece(targetSquare(event->pos())) == -1)
    {

        QByteArray pieceData = event->mimeData()->data("image/x-puzzle-piece");
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        Piece piece;
        piece.rect = targetSquare(event->pos());
        dataStream >> piece.pixmap >> piece.location;

        pieces.append(piece);

        highlightedRect = QRect();
        update(piece.rect);

        event->setDropAction(Qt::MoveAction);
        event->accept();

        if (piece.location == piece.rect.topLeft() / pieceSize())
        {
            inPlace++;
            if (inPlace == 25)
                emit puzzleCompleted();
        }
    }
    else
    {
        highlightedRect = QRect();
        event->ignore();
    }
}

void PuzzleWidget::mousePressEvent(QMouseEvent *event)
{
    QRect square = targetSquare(event->pos());
    int found = findPiece(square);

    if (found == -1)
        return;

    Piece piece = pieces.takeAt(found);

    if (piece.location == square.topLeft() / pieceSize())
        inPlace--;

    update(square);

    QByteArray itemData;
    QDataStream dataStream(&itemData, QIODevice::WriteOnly);

    dataStream << piece.pixmap << piece.location;

    QMimeData *mimeData = new QMimeData;
    mimeData->setData("image/x-puzzle-piece", itemData);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setHotSpot(event->pos() - square.topLeft());
    drag->setPixmap(piece.pixmap);

    if (drag->exec(Qt::MoveAction) == Qt::IgnoreAction)
    {
        pieces.insert(found, piece);
        update(targetSquare(event->pos()));

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
        painter.setBrush(QColor("#ffcccc"));
        painter.setPen(Qt::NoPen);
        painter.drawRect(highlightedRect.adjusted(0, 0, -1, -1));
    }

    for (const Piece &piece : pieces)
        painter.drawPixmap(piece.rect, piece.pixmap);
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

const QRect PuzzleWidget::targetSquare(const QPoint &position) const
{
    // position / pieceSize() * pieceSize() 取整左上角坐标
    return QRect(position / pieceSize() * pieceSize(), QSize(pieceSize(), pieceSize()));
}

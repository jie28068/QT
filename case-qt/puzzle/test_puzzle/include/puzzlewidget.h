#pragma once
#include <QWidget>
#include <QtWidgets>

class PuzzleWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PuzzleWidget(int size, QWidget *parent = nullptr);
    int pieceSize() const;
    int imageSize() const;
    void clear();

signals:
    void puzzleCompleted();

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    // 用户离开控件时清除高亮区域，并通知系统已经处理了拖拽事件
    void dragLeaveEvent(QDragLeaveEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    struct Piece
    {
        QPixmap pixmap;
        QRect rect;
        QPoint location;
    };
    // 判断当前位置是否有图片
    int findPiece(const QRect &pieceRect) const;
    // 确定目标方块的位置和大小
    const QRect targetSquare(const QPoint &position) const;

    QVector<Piece> pieces; // 图片容器
    QRect highlightedRect; // 高亮矩形
    int inPlace;
    int m_ImageSize;
};
/*
 * @Author: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @Date: 2024-01-05 15:17:45
 * @LastEditors: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @LastEditTime: 2024-01-16 15:05:07
 * @FilePath: \puzzle\widget\puzzlewidget.h
 */

#pragma once
#include <QWidget>
#include <QtWidgets>
#include "../include/Global.h"
using namespace MacroDf;
class PuzzleWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PuzzleWidget(int size, QWidget *parent = nullptr);
    int pieceSize() const;
    int imageSize() const;
    void clear();
signals:
    void puzzleCompleted(bool isok = true);

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    // 用户离开控件时清除高亮区域，并通知系统已经处理了拖拽事件
    void dragLeaveEvent(QDragLeaveEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    // 判断当前位置是否有图片
    int findPiece(const QRect &pieceRect) const;
    // 确定目标方块的位置和大小
    const QRect targetSquareMove(const QPoint &position) const;
    /// @brief 计算图片位置是否正确，完成标志位增加
    /// @param piece
    void addInPlace(Piece piece);
    /// @brief 检查拖拽是否合法
    /// @param position
    /// @return
    bool dragMoveIsValid(const QPoint &position);
    QVector<Piece> pieces; // 图片容器
    QRect highlightedRect; // 高亮矩形
    int inPlace;           // 完成计数位
    int m_ImageSize;       // 图片大小
};
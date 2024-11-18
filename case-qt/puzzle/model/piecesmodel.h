/*
 * @Author: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @Date: 2024-01-06 12:34:08
 * @LastEditors: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @LastEditTime: 2024-01-16 15:02:15
 */
#pragma once
#include <QAbstractListModel>
#include <QPixmap>
#include <QPoint>
#include <QStringList>
#include <QVector>
#include <QIcon>
#include <QMimeData>
#include <QRandomGenerator>
#include "../include/Global.h"

using namespace MacroDf;
class PiecesModel : public QAbstractListModel
{
    Q_OBJECT

public:
    explicit PiecesModel(int pieceSize, QObject *parent = nullptr);
    /// @brief 根据角色(role)返回不同的数据:
    /// @param index 指定模型中项目的索引
    /// @param role 指定所请求数据的角色
    /// @return
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    /// @brief  设置是否可拖拽
    /// @param index
    /// @return
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    /// @brief 删除一个连续的行范围
    /// @param row 起始行号
    /// @param count 要删除的行数
    /// @param parent
    /// @return
    bool removeRows(int row, int count, const QModelIndex &parent) override;
    /// @brief 解析拖放事件中包含的图片数据,并将图片插入模型中,处理拖放操作时接收的数据
    /// @param data
    /// @param action
    /// @param row 数据被放置的行号
    /// @param column  数据被放置的列号（对列表模型通常是无效的，因为它们只有一列）
    /// @param parent
    /// @return
    bool dropMimeData(const QMimeData *data, Qt::DropAction action,
                      int row, int column, const QModelIndex &parent) override;
    /// @brief 提供了自定义的 mime 数据,可以封装其 pixmap 和位置信息,在拖放操作中使用
    /// @param indexes
    /// @return
    QMimeData *mimeData(const QModelIndexList &indexes) const override;

    QStringList mimeTypes() const override;

    int rowCount(const QModelIndex &parent) const override;
    // int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    /// @brief 既支持复制 drop 进来的项,又支持移动 drop 进来的项。
    /// @return
    Qt::DropActions supportedDropActions() const override;

    void addPiece(const QPixmap &pixmap, const QPoint &location);
    void addPieces(const QPixmap &pixmap);
    /// @brief 设置难度切换后图片切割大小
    /// @param size
    void setPicSize(int size);

private:
    QVector<Piece> piece;
    int m_PieceSize;
};
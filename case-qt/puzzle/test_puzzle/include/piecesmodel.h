#pragma once
#include <QAbstractListModel>
#include <QPixmap>
#include <QPoint>
#include <QStringList>
#include <QVector>
#include <QIcon>
#include <QMimeData>
#include <QRandomGenerator>

class PiecesModel : public QAbstractListModel
{
    Q_OBJECT

public:
    explicit PiecesModel(int pieceSize, QObject *parent = nullptr);
    /// @brief 根据角色(role)返回不同的数据:
    /// @param index
    /// @param role
    /// @return
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    /// @brief  设置是否可拖拽
    /// @param index
    /// @return
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    /// @brief 删除一个连续的行范围
    /// @param row
    /// @param count
    /// @param parent
    /// @return
    bool removeRows(int row, int count, const QModelIndex &parent) override;
    /// @brief 解析拖放事件中包含的图片数据,并将图片插入模型中
    /// @param data
    /// @param action
    /// @param row
    /// @param column
    /// @param parent
    /// @return
    bool dropMimeData(const QMimeData *data, Qt::DropAction action,
                      int row, int column, const QModelIndex &parent) override;
    /// @brief 提供了自定义的 mime 数据,可以封装其 pixmap 和位置信息
    /// @param indexes
    /// @return
    QMimeData *mimeData(const QModelIndexList &indexes) const override;

    QStringList mimeTypes() const override;

    int rowCount(const QModelIndex &parent) const override;
    /// @brief 既支持复制 drop 进来的项,又支持移动 drop 进来的项。
    /// @return
    Qt::DropActions supportedDropActions() const override;

    void addPiece(const QPixmap &pixmap, const QPoint &location);
    void addPieces(const QPixmap &pixmap);

private:
    QVector<QPoint> locations;
    QVector<QPixmap> pixmaps;
    int m_PieceSize;
};
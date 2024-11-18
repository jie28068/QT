#include "../include/piecesmodel.h"

PiecesModel::PiecesModel(int pieceSize, QObject *parent) : QAbstractListModel(parent), m_PieceSize(pieceSize)
{
}

QVariant PiecesModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (role == Qt::DecorationRole) // 返回图标,通过pixmaps映射返回对应索引的图标,并对其进行缩放
        return QIcon(pixmaps.value(index.row()).scaled(m_PieceSize, m_PieceSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    else if (role == Qt::UserRole) // 返回原始 pixmap ,通过pixmaps映射返回
        return pixmaps.value(index.row());
    else if (role == Qt::UserRole + 1) // 返回位置信息,通过locations映射返回
        return locations.value(index.row());
    return QVariant();
}

Qt::ItemFlags PiecesModel::flags(const QModelIndex &index) const
{
    if (index.isValid())
        return (QAbstractListModel::flags(index) | Qt::ItemIsDragEnabled); // 可拖拽

    return Qt::ItemIsDropEnabled; // 接受拖拽
}

bool PiecesModel::removeRows(int row, int count, const QModelIndex &parent)
{
    if (parent.isValid())
        return false;

    if (row >= pixmaps.size() || row + count <= 0)
        return false;
    // 修剪beginRow和endRow,限制在有效范围内。
    int beginRow = qMax(0, row);
    int endRow = qMin(row + count - 1, pixmaps.size() - 1);
    // 调用beginRemoveRows()告知视图将移除行,开始行beginRow和结束行endRow。
    beginRemoveRows(parent, beginRow, endRow);
    // 循环移除
    while (beginRow <= endRow)
    {
        pixmaps.removeAt(beginRow);
        locations.removeAt(beginRow);
        ++beginRow;
    }
    // 调用endRemoveRows()告知视图完成移除行。
    endRemoveRows();
    return true;
}

bool PiecesModel::dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent)
{
    // 检查mime数据是否包含正确的格式:"image/x-puzzle-piece"
    if (!data->hasFormat("image/x-puzzle-piece"))
        return false;
    // 检查拖放操作:
    if (action == Qt::IgnoreAction)
        return true;
    // 只允许插入第一列:
    if (column > 0)
        return false;
    // 判断插入行的尾部位置endRow:
    int endRow;

    // 如果是根节点:
    if (!parent.isValid())
    {
        if (row < 0)
            endRow = pixmaps.size();
        else
            endRow = qMin(row, pixmaps.size());
    }
    else // 如果是子节点:
    {
        endRow = parent.row();
    }
    // 解析mime数据,读取 pixmap 图片和位置 location:
    QByteArray encodedData = data->data("image/x-puzzle-piece");
    QDataStream stream(&encodedData, QIODevice::ReadOnly);
    // 通过 begin/endInsertRows函数更新模型,插入数据:
    while (!stream.atEnd())
    {
        QPixmap pixmap;
        QPoint location;
        // 从数据流中读取一个 QPixmap 和一个 QPoint,分别赋值给 pixmap 和 location。
        stream >> pixmap >> location;

        beginInsertRows(QModelIndex(), endRow, endRow);
        pixmaps.insert(endRow, pixmap);
        locations.insert(endRow, location);
        endInsertRows();

        ++endRow;
    }

    return true;
}

QMimeData *PiecesModel::mimeData(const QModelIndexList &indexes) const
{
    QMimeData *mimeData = new QMimeData();
    // 保存数据的形式
    QByteArray encodedData;
    QDataStream stream(&encodedData, QIODevice::WriteOnly);

    for (const QModelIndex &index : indexes)
    {
        if (index.isValid())
        {
            QPixmap pixmap = qvariant_cast<QPixmap>(data(index, Qt::UserRole));
            QPoint location = data(index, Qt::UserRole + 1).toPoint();
            stream << pixmap << location;
        }
    }
    // 在 drop 操作时,则可以通过读取 mimeData 中"image/x-puzzle-piece"的数据
    mimeData->setData("image/x-puzzle-piece", encodedData);
    return mimeData;
}

QStringList PiecesModel::mimeTypes() const
{
    QStringList types;
    types << "image/x-puzzle-piece";
    return types;
}

int PiecesModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : pixmaps.size();
}

Qt::DropActions PiecesModel::supportedDropActions() const
{
    return Qt::CopyAction | Qt::MoveAction;
}

void PiecesModel::addPiece(const QPixmap &pixmap, const QPoint &location)
{
    // 生成随机的row插入。
    int row;
    // 生成0或pixmaps.size(),也即头部或尾部插入
    if (QRandomGenerator::global()->bounded(2) == 1)
        row = 0;
    else
        row = pixmaps.size();
    // 调用beginInsertRows()和endInsertRows(),告知视图有数据将被插入
    beginInsertRows(QModelIndex(), row, row);
    pixmaps.insert(row, pixmap);
    locations.insert(row, location);
    endInsertRows();
}

void PiecesModel::addPieces(const QPixmap &pixmap)
{
    // 如果已经有数据,先清除原有数据。
    if (!pixmaps.isEmpty())
    {
        beginRemoveRows(QModelIndex(), 0, pixmaps.size() - 1);
        pixmaps.clear();
        locations.clear();
        endRemoveRows();
    }
    // 制作每一片拼图片段。
    for (int y = 0; y < 4; ++y)
    {
        for (int x = 0; x < 4; ++x)
        {
            QPixmap pieceImage = pixmap.copy(x * m_PieceSize, y * m_PieceSize, m_PieceSize, m_PieceSize);
            addPiece(pieceImage, QPoint(x, y));
        }
    }
}

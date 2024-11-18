// :/images/icons/check2.ico
#ifndef MAINWINDOWS_H
#define MAINWINDOWS_H

#include <QToolTip>
#include <QApplication>
#include <QFileSystemModel>
#include <QListView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QModelIndex>
#include <QMenu>
#include <QAction>
#include <QInputDialog>
#include <QContextMenuEvent>
#include <QDebug>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QFileInfo>
#include <QDateTime>
#include <QLabel>
#include <QListWidget>

#include <QMimeData>
#include <QDrag>
#include <QDropEvent>
#include <QMessageBox>

#include <QDebug>

class FileListWidget : public QListWidget
{
public:
    FileListWidget(const QString &dir, QWidget *parent = nullptr);
    void refresh(const QString &dir);
};

class FileListView : public QListView
{
public:
    FileListView(QWidget *parent = nullptr) : QListView(parent) {}

protected:
    void mousePressEvent(QMouseEvent *event) override;

    void dragEnterEvent(QDragEnterEvent *event) override;

    void dragMoveEvent(QDragMoveEvent *event) override;

    void dropEvent(QDropEvent *event) override;
    /// @brief 检测是否为文件夹
    /// @param position
    /// @return
    bool checkIfDropOnFolder(const QPoint &position);
};

class CustomFileSystemModel : public QFileSystemModel
{
public:
    CustomFileSystemModel(QObject *parent) : QFileSystemModel(parent) {}
    Qt::ItemFlags flags(const QModelIndex &index) const override
    {
        Qt::ItemFlags defaultFlags = QFileSystemModel::flags(index);
        QFile::Permissions permissions = this->permissions(index);

        // 这里我们假设如果一个文件不可写入，则用户也无法执行删除等操作
        if (!(permissions & QFile::WriteUser))
        {
            defaultFlags &= ~(Qt::ItemIsEditable | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        }

        return defaultFlags;
    }

    // TODO: 可能还需要重载其他方法来处理
};

class FileExplorer : public QWidget
{
public:
    FileExplorer(QWidget *parent = nullptr);
    void onDoubleClicked(const QModelIndex &index);
    /// @brief 复制粘贴目录
    /// @param src
    /// @param dst
    void copyPath(QString src, QString dst);

protected:
    void onTreeContextMenu(const QPoint &pos);
    bool eventFilter(QObject *watched, QEvent *event) override;
    /// @brief 检测是否有同名
    /// @param directory 目的文件路径
    /// @param baseName 文件名
    /// @param extension 文件后缀
    /// @return
    QString generateUniqueFileName(const QString &directory, const QString &baseName, const QString &extension);

private slots:
    void onitemClicked(QListWidgetItem *item);
    /// @brief 重命名
    /// @param index
    void renameFileOrFolder();
    /// @brief 删除
    /// @param index
    void deleteFileOrFolder();
    /// @brief 复制
    void copyFileOrFolder();
    /// @brief 粘贴
    void pasteFileOrFolder();
    /// @brief 返回
    void goBack();
    /// @brief 更新路径
    /// @param index
    void updatePath(const QModelIndex &index);
    /// @brief 剪切
    /// @param index
    void cutFileOrFolder();
    /// @brief 新建文件
    void newFile();
    /// @brief 新建文件夹
    void newFolder();
    /// @brief 全选
    void selectAll(int state);

private:
    FileListView *listView;
    QFileSystemModel *model;
    QPushButton *backButton;
    FileListWidget *fileList;
    QStringList listOfFilesToCopy;
    QAction *pasteAction;
    bool cutOperation; // 剪切状态
};
#endif
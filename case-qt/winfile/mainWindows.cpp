#include "mainWindows.h"
#include <QCheckBox>
#include <QDesktopServices>
#include <QFileDialog>

#define COPYS "-副本"
#define DECOLLATOR '/'
#define ARROWS ">"
#define DOT '.'
#define HINTMESSAGE(Test) \
    QMessageBox::information(&QWidget(), "", Test)

FileExplorer::FileExplorer(QWidget *parent) : QWidget(parent), cutOperation(false)
{
    resize(600, 480);
    // 创建文件系统模型
    model = new CustomFileSystemModel(this);
    model->setRootPath(QDir::homePath());
    model->setFilter(QDir::AllEntries | QDir::NoDotAndDotDot | QDir::AllDirs); // 显示所有条目，但不显示"."和".."条目

    // 创建ListView并设置其模型
    listView = new FileListView(this);
    listView->setModel(model);
    listView->setRootIndex(model->index(QDir::homePath()));
    // 拖放功能
    listView->setDragDropMode(QAbstractItemView::DragDrop);           // 拖动操作会移动项目
    listView->setSelectionMode(QAbstractItemView::ExtendedSelection); // 选择多个项目
    listView->setDragEnabled(true);                                   // 启用了列表视图的拖动功能
    listView->setAcceptDrops(true);                                   // 可以接受拖放操作
    listView->setDropIndicatorShown(true);                            // 拖动的项目将在哪个位置被放下
    // 设置文件系统视图为大图标模式
    listView->setViewMode(QListView::IconMode);
    listView->setResizeMode(QListView::Adjust);
    listView->setSpacing(20);
    listView->setIconSize(QSize(48, 48));
    listView->setGridSize(QSize(80, 80));
    listView->installEventFilter(this);

    // 导航栏按钮
    QWidget *widget = new QWidget(this);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QHBoxLayout *layout1 = new QHBoxLayout(widget);
    QCheckBox *checkBox1 = new QCheckBox(widget);
    checkBox1->setText(tr("select all"));
    QPushButton *pushButtonCut = new QPushButton(tr("cut"), widget);
    QPushButton *pushButtonDel = new QPushButton(tr("delete"), widget);
    QPushButton *pushButtonRename = new QPushButton(tr("rename"), widget);
    backButton = new QPushButton("<", this);
    backButton->setFixedSize(20, 20);
    // QSpacerItem *spacer = new QSpacerItem(50, 0, QSizePolicy::Expanding, QSizePolicy::Maximum);
    // 导航栏
    fileList = new FileListWidget(QDir::homePath(), this);
    layout1->addWidget(checkBox1);
    layout1->addWidget(pushButtonCut);
    layout1->addWidget(pushButtonDel);
    layout1->addWidget(pushButtonRename);
    layout1->addWidget(backButton);
    layout1->addWidget(fileList);
    widget->setLayout(layout1);
    // 创建Layout并添加控件
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(widget);
    layout->addWidget(listView);

    listView->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(backButton, &QPushButton::clicked, this, &FileExplorer::goBack);
    connect(listView, &QListView::doubleClicked, this, &FileExplorer::onDoubleClicked);
    connect(fileList, &QListWidget::itemClicked, this, &FileExplorer::onitemClicked);
    connect(listView, &QListView::customContextMenuRequested, this, &FileExplorer::onTreeContextMenu);
    connect(pushButtonCut, &QPushButton::clicked, this, &FileExplorer::cutFileOrFolder);
    connect(pushButtonDel, &QPushButton::clicked, this, &FileExplorer::deleteFileOrFolder);
    connect(pushButtonRename, &QPushButton::clicked, this, &FileExplorer::renameFileOrFolder);
    connect(checkBox1, &QCheckBox::stateChanged, this, &FileExplorer::selectAll);
}

void FileExplorer::onDoubleClicked(const QModelIndex &index)
{
    if (model->isDir(index))
    {
        listView->setRootIndex(index);
        updatePath(index);
    }
    else
    {
        QString filePath = model->filePath(index);
        // 处理文件的打开逻辑
        QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
    }
}

void FileExplorer::onTreeContextMenu(const QPoint &pos)
{
    QModelIndex index = listView->indexAt(pos);
    if (index.isValid())
    {
        QMenu contextMenu;
        QAction *cutAction = contextMenu.addAction(tr("Cut"));
        QAction *copyAction = contextMenu.addAction(tr("Copy"));
        pasteAction = contextMenu.addAction(tr("Paste"));
        QAction *deleteAction = contextMenu.addAction(tr("Delete"));
        QAction *renameAction = contextMenu.addAction(tr("Rename"));

        cutAction->setShortcut(QKeySequence::Cut);
        copyAction->setShortcut(QKeySequence::Copy);
        deleteAction->setShortcut(QKeySequence::Delete);
        pasteAction->setShortcut(QKeySequence::Paste);

        if (listOfFilesToCopy.isEmpty())
            pasteAction->setDisabled(true);

        QAction *selectedAction = contextMenu.exec(cursor().pos());
        if (selectedAction == renameAction)
            renameFileOrFolder();
        else if (selectedAction == deleteAction)
            deleteFileOrFolder();
        else if (selectedAction == copyAction)
            copyFileOrFolder();
        else if (selectedAction == pasteAction)
            pasteFileOrFolder();
        else if (selectedAction == cutAction)
            cutFileOrFolder();
    }
    else
    {
        QMenu contextMenu;
        QAction *fileAction = contextMenu.addAction(tr("new file"));
        QAction *folderAction = contextMenu.addAction(tr("new folder"));
        pasteAction = contextMenu.addAction(tr("Paste"));
        if (listOfFilesToCopy.isEmpty())
            pasteAction->setDisabled(true);
        QAction *selectedAction = contextMenu.exec(cursor().pos());
        if (selectedAction == pasteAction)
            pasteFileOrFolder();
        else if (selectedAction == fileAction)
            newFile();
        else if (selectedAction == folderAction)
            newFolder();
    }
}

void FileExplorer::deleteFileOrFolder()
{
    QModelIndexList indexes = listView->selectionModel()->selectedIndexes();
    for (QModelIndex index : indexes)
    {
        if (model->fileInfo(index).isDir())
        {
            QString filePath = model->filePath(index);
            QDir dir(filePath);
            if (dir.exists())
            {
                dir.removeRecursively();
            }
        }
        else
        {
            model->remove(index);
        }
    }
    listView->clearSelection();
}

void FileExplorer::copyFileOrFolder()
{
    cutOperation = false;
    QModelIndexList indexes = listView->selectionModel()->selectedIndexes();
    listOfFilesToCopy.clear();
    for (QModelIndex index : indexes)
    {
        QString filePath = model->filePath(index);
        listOfFilesToCopy.append(filePath);
    }
    listView->clearSelection();
}

void FileExplorer::copyPath(QString src, QString dst)
{
    QDir dir(src);
    if (!dir.exists())
        return;

    for (QString d : dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot))
    {
        QString dst_path = dst + DECOLLATOR + d;
        dir.mkpath(dst_path);
        copyPath(src + DECOLLATOR + d, dst_path);
    }

    for (QString f : dir.entryList(QDir::Files))
    {
        QFile::copy(src + DECOLLATOR + f, dst + DECOLLATOR + f);
    }
}

QString FileExplorer::generateUniqueFileName(const QString &directory, const QString &baseName, const QString &extension)
{
    QFileInfo fileInfo;
    // 设置初始文件名
    QString uniqueName = baseName + DOT + extension;
    int counter = 1; // 设置文件名称后缀计数器
    do
    {
        fileInfo.setFile(directory + DECOLLATOR + uniqueName);
        if (fileInfo.exists())
        {
            // 如果文件已存在，生成带有计数器的新文件名
            uniqueName = QString("%1_%2.%3").arg(baseName).arg(counter++).arg(extension);
        }
        else
        {
            // 文件名是唯一的，可以使用
            break;
        }
    } while (true);

    return directory + DECOLLATOR + uniqueName;
}

void FileExplorer::pasteFileOrFolder()
{
    if (listOfFilesToCopy.size() > 0)
    {
        pasteAction->setEnabled(true);
    }
    QString destDir = model->filePath(listView->rootIndex());
    for (QString filePath : listOfFilesToCopy)
    {
        QFileInfo fileInfo(filePath);
        QString destPath = QString();                                                               // 目标全称
        QString fileName = fileInfo.fileName();                                                     // 文件名
        QString filedir = fileInfo.path();                                                          // 源路径
        QString destinationPath = QDir(destDir).filePath(fileName);                                 // 目标全称
        bool isCovered = false;                                                                     // 是否覆盖
        destPath = generateUniqueFileName(destDir, fileInfo.completeBaseName(), fileInfo.suffix()); // 目标拼接
        if (filedir != destDir)                                                                     // 当前路径相同时复制不做提示，直接加上副本
        {
            if (QFile::exists(destinationPath))
            {
                // 如果文件已经存在，可以对话框确认覆盖或跳过
                QMessageBox::StandardButton btn = QMessageBox::question(this, tr("Confirm Overwrite"),
                                                                        tr("The following files already exist:\n%1\nDo you want to overwrite them?").arg(destinationPath),
                                                                        QMessageBox::Yes | QMessageBox::No);
                if (btn == QMessageBox::Yes)
                {
                    isCovered = true;
                    destPath = destinationPath;
                }
                else
                {
                    isCovered = false;
                }
            }
            else
            {
                destPath = destinationPath;
            }
        }

        if (filedir == destDir && cutOperation) // 路径相同时剪切不做处理
        {
            continue;
        }

        if (fileInfo.isDir())
        {
            // 要复制目录内容，需要递归地复制所有文件和子目录
            copyPath(filePath, destPath);
        }
        else
        {
            // 若路径不存在则创建不存在的文件,不然无法复制粘贴成功
            QDir path(destDir);
            if (path.exists())
            {
                QFile file(filePath);
                if (isCovered)
                {
                    file.remove(destPath);
                }
                file.copy(filePath, destPath);
            }
            else
            {
                path.mkdir(destDir);
                QFile file(filePath);
                file.copy(filePath, destPath);
            }
        }

        if (cutOperation) // 剪切后删除以前的内容
        {
            QFileInfo fileInfo(filePath);
            if (fileInfo.isDir())
            {
                QDir dir(filePath);
                if (dir.exists())
                {
                    dir.removeRecursively();
                }
            }
            else
            {
                QFile file(filePath);
                file.remove();
            }
        }
    }

    if (cutOperation)
    {
        listOfFilesToCopy.clear();
        pasteAction->setEnabled(false);
    }
    listView->clearSelection();
}
void FileExplorer::goBack()
{
    QModelIndex currentIndex = listView->rootIndex();
    QModelIndex parentIndex = model->parent(currentIndex);
    if (parentIndex.isValid())
    {
        listView->setRootIndex(parentIndex);
        fileList->refresh(model->filePath(parentIndex));
    }
}

void FileExplorer::updatePath(const QModelIndex &index)
{
    QString currentPath = model->filePath(index);
    fileList->refresh(currentPath);
    listView->setRootIndex(index);
}

void FileExplorer::cutFileOrFolder()
{
    cutOperation = true;
    QModelIndexList indexes = listView->selectionModel()->selectedIndexes();
    listOfFilesToCopy.clear();
    for (QModelIndex index : indexes)
    {
        QString filePath = model->filePath(index);
        listOfFilesToCopy.append(filePath);
    }
    listView->clearSelection();
}

void FileExplorer::newFile()
{
    QString parentPath = model->filePath(listView->rootIndex());
    QString fileName = QFileDialog::getSaveFileName(nullptr,
                                                    tr("Create File"), parentPath, "Text Files (*.txt);;All Files (*)");
    if (!fileName.isEmpty())
    {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly))
        {
            HINTMESSAGE(tr("Failed to create file"));
        }
        else
        {
            file.close();
        }
    }
}

void FileExplorer::newFolder()
{
    QString newName = QInputDialog::getText(this, tr("New Folder"), tr("name:"));
    if (newName.isEmpty())
        return;
    QString parentPath = model->filePath(listView->rootIndex());
    QDir dir;
    if (!dir.exists(parentPath + DECOLLATOR + newName))
    {
        if (dir.mkdir(parentPath + DECOLLATOR + newName))
        {
        }
        else
        {
            HINTMESSAGE(tr("Failed to create directory"));
        }
    }
    else
    {
        HINTMESSAGE(tr("Directory already exists"));
    }
}

void FileExplorer::selectAll(int state)
{
    if (state == Qt::Checked)
    {
        listView->selectAll();
    }
    else
    {
        listView->clearSelection();
    }
}

void FileExplorer::renameFileOrFolder()
{
    QModelIndexList indexes = listView->selectionModel()->selectedIndexes();
    listOfFilesToCopy.clear();
    for (QModelIndex index : indexes)
    {
        QString filePath = model->filePath(index);
        QFileInfo fileInfo(filePath);
        QFile file(filePath);
        QString newName = QInputDialog::getText(this, tr("Rename"), tr("New name:"));
        if (newName.isEmpty())
            return;
        QString parentPath = model->filePath(listView->rootIndex());
        auto newname = parentPath + DECOLLATOR + newName + DOT + fileInfo.suffix();
        file.rename(file.fileName(), newname);
    }
    listView->clearSelection();
}

qint64 calculateFolderSize(const QString &folderPath)
{
    QDir folder(folderPath);
    QFileInfoList fileInfoList = folder.entryInfoList(QDir::AllEntries | QDir::NoDotAndDotDot);
    qint64 totalSize = 0;

    for (const QFileInfo &fileInfo : fileInfoList)
    {
        if (fileInfo.isFile())
        {
            totalSize += fileInfo.size();
        }
        else if (fileInfo.isDir())
        {
            totalSize += calculateFolderSize(fileInfo.absoluteFilePath());
        }
    }

    return totalSize / 1024;
}

bool FileExplorer::eventFilter(QObject *watched, QEvent *event)
{
    // 验证事件是针对 listView 的
    if (watched == listView && event->type() == QEvent::ToolTip)
    {
        QHelpEvent *helpEvent = static_cast<QHelpEvent *>(event);
        if (helpEvent)
        {
            QModelIndex index = listView->indexAt(helpEvent->pos());
            if (index.isValid())
            {
                QString toolTipText;
                QLocale locale;
                // 获取项的完整路径
                QString filePath = model->filePath(index);
                QFileInfo fileInfo(filePath);
                QString longDate = locale.toString(fileInfo.lastModified(), QLocale::LongFormat);
                toolTipText = QString(tr("Name: %1\nSize: %2KB\nType: %3\nLast Modified: %4"))
                                  .arg(fileInfo.fileName())
                                  .arg(!fileInfo.isFile() ? calculateFolderSize(filePath) : fileInfo.size() / 1024)
                                  .arg(!fileInfo.isFile() ? tr("file") : fileInfo.suffix())
                                  .arg(longDate);

                // 显示工具提示
                QToolTip::showText(helpEvent->globalPos(), toolTipText);
            }
            else
            {
                QToolTip::hideText();
                event->ignore();
            }
            return true; // 事件已处理
        }
    }
    // 如果不是自己处理的事件，调用基类的事件过滤器
    return QWidget::eventFilter(watched, event);
}

FileListWidget::FileListWidget(const QString &dir, QWidget *parent) : QListWidget(parent)
{
    this->setFlow(QListWidget::LeftToRight);
    this->setSelectionMode(QListWidget::SingleSelection);
    this->setFixedHeight(24);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->setViewMode(QListView::ListMode);
    this->setResizeMode(QListView::Adjust);

    setFrameShape(QFrame::NoFrame); // 设置无边框
    auto initlist = dir.split("/");
    for (auto &str : initlist)
    {
        QListWidgetItem *item = new QListWidgetItem(str, this);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        addItem(ARROWS);
        addItem(item);
    }
}

void FileListWidget::refresh(const QString &dir)
{
    this->clear();
    auto initlist = dir.split("/");
    for (auto &str : initlist)
    {
        QListWidgetItem *item = new QListWidgetItem(str, this);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        addItem(ARROWS);
        addItem(item);
    }
}

void FileExplorer::onitemClicked(QListWidgetItem *item)
{
    if (!item)
        return;
    auto row = fileList->row(item) + 1;
    QString strdir;
    for (int i = 0; i < row; ++i)
    {
        QListWidgetItem *itemc = fileList->item(i);
        if (itemc->text() != ARROWS)
        {
            strdir.push_back(QString(itemc->text() + "/"));
        }
    }
    strdir.chop(1);
    if (!strdir.isEmpty())
    {
        fileList->refresh(strdir);
        model->setRootPath(strdir);
        listView->setRootIndex(model->index(strdir));
    }
}

void FileListView::mousePressEvent(QMouseEvent *event)
{
    QModelIndex index = indexAt(event->pos());
    if (!index.isValid())
    {
        clearSelection();
    }
    else
    {
        QListView::mousePressEvent(event);
    }
}

void FileListView::dragEnterEvent(QDragEnterEvent *event)
{
    // event->setAccepted(checkIfDropOnFolder(event->pos()));
    event->setAccepted(true);
}

void FileListView::dragMoveEvent(QDragMoveEvent *event)
{
    event->setAccepted(checkIfDropOnFolder(event->pos()));
}

void FileListView::dropEvent(QDropEvent *event)
{
    if (event->mimeData()->hasUrls() /*&& checkIfDropOnFolder(event->pos())*/)
    {
        QModelIndex index = indexAt(event->pos());
        if (!index.isValid())
            return;

        QString targetPath = model()->data(index, QFileSystemModel::FilePathRole).toString();
        QFileInfo targetInfo(targetPath);
        if (!targetInfo.isDir())
        {
            event->ignore();
            return;
        }

        QList<QUrl> urls = event->mimeData()->urls();
        for (const QUrl &url : urls)
        {
            QString sourcePath = url.toLocalFile();
            QString fileName = QFileInfo(sourcePath).fileName();
            QString destinationPath = QDir(targetPath).filePath(fileName);

            // 移动或复制文件到目标文件夹中
            if (QFile::exists(destinationPath))
            {
                // 如果文件已经存在，可以对话框确认覆盖或跳过
                QMessageBox::StandardButton btn = QMessageBox::question(this, tr("Confirm Overwrite"),
                                                                        tr("The following files already exist:\n%1\nDo you want to overwrite them?").arg(destinationPath),
                                                                        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
                if (btn == QMessageBox::Yes)
                {
                    QFile::remove(destinationPath);
                    QFile::rename(sourcePath, destinationPath);
                }
                else if (btn == QMessageBox::No)
                {
                    // 用户选择跳过，不做任何操作
                }
                else
                {
                    // 用户选择取消，取消整个拖放操作
                    event->ignore();
                    return;
                }
            }
            else
            {
                QFile::rename(sourcePath, destinationPath);
            }
        }
    }
    else
    {
        event->ignore();
    }
}

bool FileListView::checkIfDropOnFolder(const QPoint &position)
{
    QModelIndex index = indexAt(position);
    if (!index.isValid())
        return false;

    QFileInfo fileInfo(model()->data(index, QFileSystemModel::FilePathRole).toString());
    return fileInfo.isDir();
}

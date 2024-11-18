#include "../include/mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setupMenus();
    setupWidgets();
    model = new PiecesModel(puzzleWidget->pieceSize(), this);
    piecesList->setModel(model);

    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    setWindowTitle(tr("Puzzle"));
}

void MainWindow::setupMenus()
{
    // 添加action到菜单， 菜单到菜单栏
    QMenu *fileMenu = menuBar()->addMenu("Text");

    QAction *openAction = fileMenu->addAction("image", this, &MainWindow::openImage);

    openAction->setShortcuts(QKeySequence::Open);

    QAction *eixtAction = fileMenu->addAction("quit", this, &QCoreApplication::exit);

    eixtAction->setShortcuts(QKeySequence::Quit);

    QMenu *resrtMenu = menuBar()->addMenu("game");

    QAction *resrtAction = resrtMenu->addAction("rest", this, &MainWindow::setupPuzzle);
}

void MainWindow::setupWidgets()
{
    QFrame *frame = new QFrame;
    QHBoxLayout *frameLayout = new QHBoxLayout(frame);

    puzzleWidget = new PuzzleWidget(400);

    piecesList = new QListView;
    // 允许拖动项
    piecesList->setDragEnabled(true);
    // 设置为图标模式
    piecesList->setViewMode(QListView::IconMode);
    // 设置图标大小
    piecesList->setIconSize(QSize(puzzleWidget->pieceSize() - 20, puzzleWidget->pieceSize() - 20));
    // 设置网格大小
    piecesList->setGridSize(QSize(puzzleWidget->pieceSize(), puzzleWidget->pieceSize()));
    // 设置项之间的间隔
    piecesList->setSpacing(10);
    // 设置项移动方式为粘贴定位
    piecesList->setMovement(QListView::Snap);
    // 允许将项拖放到该部件
    piecesList->setAcceptDrops(true);
    // 显示可拖动放置指示器
    piecesList->setDropIndicatorShown(true);

    PiecesModel *models = new PiecesModel(puzzleWidget->pieceSize(), this);
    piecesList->setModel(models);

    connect(puzzleWidget, &PuzzleWidget::puzzleCompleted,
            this, &MainWindow::setCompleted, Qt::QueuedConnection);

    frameLayout->addWidget(piecesList);
    frameLayout->addWidget(puzzleWidget);
    setCentralWidget(frame);
}

void MainWindow::openImage()
{
#if 1 // 复杂方法
    // 获取用户默认的图片使用目录
    const QString directory = QStandardPaths::standardLocations(QStandardPaths::PicturesLocation).value(0, QDir::homePath());

    QFileDialog dialog(this, "open image", directory);

    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    // 设置文件模式为ExistingFile,只能选择已存在的文件
    dialog.setFileMode(QFileDialog::ExistingFile);
    // 获取QImageReader支持的所有mimetype(媒体类型),添加到QStringList中
    QStringList mimeTypeFilters;
    for (const QByteArray &mimeTypeName : QImageReader::supportedMimeTypes())
    {
        mimeTypeFilters.append(mimeTypeName);
    }
    mimeTypeFilters.sort();
    // 将QStringList设置为QFileDialog的ImageFilter,这样QFileDialog只显示支持的图片类型。
    dialog.setMimeTypeFilters(mimeTypeFilters);
    dialog.selectMimeTypeFilter("image/jpeg");
    if (dialog.exec() == QDialog::Accepted)
    {
        // 只要选择的第一张图片
        loadImage(dialog.selectedFiles().constFirst());
    }
#else // 简单方式
    const QString filePath = QFileDialog::getOpenFileName(&QWidget(), "打开图片", "", "JPG(*.jpg)");
    loadImage(filePath);
#endif
}

void MainWindow::setupPuzzle()
{
    // 计算新的图像大小,取原始图片宽高的最小值作为新的尺寸size
    int size = qMin(puzzleImage.width(), puzzleImage.height());
    // 从原始图片中剪切出一个大小为 sizexsize 的部分作为新的puzzleImage,
    // 重新调整新的puzzleImage大小为puzzleWidget的imageSize,使用Qt::SmoothTransformation平滑缩放
    puzzleImage = puzzleImage.copy((puzzleImage.width() - size) / 2, (puzzleImage.height() - size) / 2, size, size)
                      .scaled(puzzleWidget->imageSize(), puzzleWidget->imageSize(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    model->addPieces(puzzleImage);
    puzzleWidget->clear();
}

void MainWindow::loadImage(const QString &fileName)
{
    QPixmap newImage;
    if (!newImage.load(fileName))
    {
        QMessageBox::warning(this, tr("open image"),
                             tr("image error!"),
                             QMessageBox::Close);
        return;
    }
    puzzleImage = newImage;
    setupPuzzle();
}

void MainWindow::setCompleted()
{
    QMessageBox::information(this, tr("Puzzle Completed"),
                             tr("Congratulations! You have completed the puzzle!\n"
                                "Click OK to start again."),
                             QMessageBox::Ok);
    setupPuzzle();
}
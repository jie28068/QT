#include "mainwindow.h"
#include "../include/Global.h"

MacroDf::Varable *MacroDf::mVarable = nullptr;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    MacroDf::mVarable = new MacroDf::Varable;
    setupbtn();
    setupWidgets();
    originalIamgeDialog();

    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    setWindowTitle(tr("你拼我拼"));
}

void MainWindow::viewOriginalImage()
{
    if (labelImage)
    {
        labelImage->setPixmap(puzzleImage);
        dialog->show();
    }
}

void MainWindow::timeUpdate()
{
    if (countDownTimerValue == 0)
    {
        countDownTimer->stop();
        setCompleted(false);
    }
    else
    {
        countDownTimerValue--;
        timerLabel->setText("<font color='black'>时间还剩[</font><font color='red'>" + QString::number(countDownTimerValue) + "</font><font color='black'>]秒！</font>");
    }
}

void MainWindow::setupbtn()
{
    QTextEdit *textEdit = new QTextEdit;
    QFile file(":/images/help");
    if (file.open(QFile::ReadOnly | QFile::Text))
    {
        QTextStream stream(&file);
        stream.setCodec("UTF-8"); // 设置正确的编码格式
        QString markdownContent = stream.readAll();
        // 自 Qt 5.14 起，QTextEdit 支持 Markdown
        textEdit->setMarkdown(markdownContent);
        textEdit->setReadOnly(true);
    }
    // 添加action到菜单栏
    QAction *beginAction = new QAction("开始游戏", this);
    QAction *eixtAction = new QAction("结束游戏", this);
    eixtAction->setShortcuts(QKeySequence::Quit);
    QAction *helpAction = new QAction("帮助", this);
    QToolBar *hToolBar = new QToolBar(this);
    hToolBar->addAction(beginAction);
    hToolBar->addAction(eixtAction);
    hToolBar->addAction(helpAction);
    addToolBar(Qt::TopToolBarArea, hToolBar);

    connect(beginAction, &QAction::triggered, this, &MainWindow::beginGame);
    connect(eixtAction, &QAction::triggered, this, &QCoreApplication::exit);
    connect(helpAction, &QAction::triggered, [=]()
            { QDialog  *dia = new QDialog(this);
            dia->setWindowTitle("帮助");
            QHBoxLayout*layout  = new QHBoxLayout(dia);
            layout->addWidget(textEdit);
                dia->setLayout(layout); 
                dia->show(); });
}

void MainWindow::setupWidgets()
{
    QFrame *frame = new QFrame;
    QHBoxLayout *frameLayout = new QHBoxLayout(frame);

    puzzleWidget = new PuzzleWidget(MacroDf::widgetSize);

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

    model = new PiecesModel(puzzleWidget->pieceSize(), this);
    piecesList->setModel(model);

    frameLayout->addWidget(piecesList);
    frameLayout->addWidget(puzzleWidget);
    setCentralWidget(frame);

    //  底部按钮
    // 计时器
    countDownTimer = new QTimer(this);
    countDownTimer->setInterval(1000); // 设置定时器每1000毫秒触发一次
    countDownTimer->stop();
    QPushButton *restorebtn = new QPushButton("重置", this);
    QPushButton *viewbtn = new QPushButton("原图", this);
    timerLabel = new QLabel("时间还剩[]秒！", this);
    timerLabel->setVisible(false);
    statusBar()->addWidget(viewbtn);
    statusBar()->addWidget(restorebtn);
    statusBar()->addWidget(timerLabel);
    // end

    connect(countDownTimer, &QTimer::timeout, this, &MainWindow::timeUpdate);
    connect(restorebtn, &QPushButton::clicked, this, &MainWindow::setupPuzzle);
    connect(viewbtn, &QPushButton::clicked, this, &MainWindow::viewOriginalImage);
    connect(puzzleWidget, &PuzzleWidget::puzzleCompleted,
            this, &MainWindow::setCompleted, Qt::QueuedConnection);
}

void MainWindow::originalIamgeDialog()
{
    dialog = new QDialog(this);
    dialog->setWindowTitle("查看原图");
    QHBoxLayout *layout = new QHBoxLayout(dialog);
    labelImage = new QLabel(dialog);
    dialog->setFixedSize(MacroDf::widgetSize, MacroDf::widgetSize);
    labelImage->setPixmap(puzzleImage);
    layout->addWidget(labelImage);
    dialog->setLayout(layout);
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
    QPixmap pixmap = puzzleImage;
    if (MacroDf::mVarable != nullptr)
    {
        if (MacroDf::mVarable->m_model == Model::customsPass)
        {
            timerLabel->setVisible(true);
            countDownTimer->start();
            if (MacroDf::mVarable->m_lists)
            {
                pixmap = MacroDf::mVarable->m_lists->at(MacroDf::mVarable->levelDesignCount)->pixmap;
                countDownTimerValue = MacroDf::mVarable->m_lists->at(MacroDf::mVarable->levelDesignCount)->time;
            }
        }
        else if (MacroDf::mVarable->m_model == Model::relaxation)
        { // 悠闲模式
            timerLabel->setVisible(false);
            countDownTimer->stop();
            if (MacroDf::mVarable->m_relax)
            {
                pixmap = MacroDf::mVarable->m_relax->pixmap;
            }
        }
    }
    //  计算新的图像大小,取原始图片宽高的最小值作为新的尺寸size
    int size = qMin(pixmap.width(), pixmap.height());
    // 从原始图片中剪切出一个大小为 sizexsize 的部分作为新的puzzleImage,
    // 重新调整新的puzzleImage大小为puzzleWidget的imageSize,使用Qt::SmoothTransformation平滑缩放
    pixmap = pixmap.copy((pixmap.width() - size) / 2, (pixmap.height() - size) / 2, size, size)
                 .scaled(puzzleWidget->imageSize(), puzzleWidget->imageSize(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    // 设置图标大小
    piecesList->setIconSize(QSize(puzzleWidget->pieceSize() - 20, puzzleWidget->pieceSize() - 20));
    // 设置网格大小
    piecesList->setGridSize(QSize(puzzleWidget->pieceSize(), puzzleWidget->pieceSize()));
    // 调整widget宽度，从而控制列数
    piecesList->setFixedWidth((piecesList->gridSize().width() + 20) * MacroDf::getCloum());
    // 重新设置listview图片大小
    model->setPicSize(puzzleWidget->pieceSize());
    // end
    model->addPieces(pixmap);
    puzzleImage = pixmap;
    puzzleWidget->clear();
}

void MainWindow::loadImage(const QString &fileName)
{
    QPixmap newImage;
    if (!newImage.load(fileName))
    {
        QMessageBox::warning(this, tr("提示"),
                             tr("打开图片失败!"),
                             QMessageBox::Close);
        return;
    }
    puzzleImage = newImage;
    setupPuzzle();
}

void MainWindow::beginGame()
{
    MacroDf::mVarable->levelDesignCount = 0; // 关卡数重新计算
    beginwidget = new Beginwidget(this);
    connect(beginwidget, &Beginwidget::beginGame, this, &MainWindow::setupPuzzle);
}

void MainWindow::setCompleted(bool iscompleted)
{
    QString str = QString();
    if (iscompleted)
    {
        if (MacroDf::mVarable->levelDesignCount == 3)
        {
            str = "好家伙！让你玩通关了！";
        }
        else
        {
            str = "小天才！恭喜你完成这张拼图！";
        }
    }
    else
    {
        str = "小老弟！汗流浃背了吧！";
    }
    countDownTimer->stop();
    QMessageBox *messagebox = new QMessageBox(this);
    messagebox->setWindowTitle("来自迪迦的话");
    messagebox->setText(str);
    QPushButton *againButton = messagebox->addButton("再试一次", QMessageBox::NoRole);
    QPushButton *switchButton = messagebox->addButton("切换模式", QMessageBox::YesRole);
    if (iscompleted && MacroDf::mVarable->m_model == Model::customsPass && MacroDf::mVarable->levelDesignCount < 3)
    {
        QPushButton *nextButton = messagebox->addButton("下一关", QMessageBox::ApplyRole);
        connect(nextButton, &QPushButton::clicked, [=]()
                { MacroDf::mVarable->levelDesignCount++;
                setupPuzzle(); });
    }

    connect(switchButton, &QPushButton::clicked, [=]()
            { beginGame(); });
    connect(againButton, &QPushButton::clicked, [=]()
            { setupPuzzle(); });
    messagebox->show();
}
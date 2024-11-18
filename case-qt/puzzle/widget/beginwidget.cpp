#include "beginwidget.h"

Beginwidget::Beginwidget(QWidget *parent) : QDialog(parent)
{
    initUI();
}

Beginwidget::~Beginwidget()
{
}

void Beginwidget::initUI()
{
    setWindowTitle("选择模式");
    setFixedSize(400, 200); // 设置弹窗固定大小

    auto mainLayout = new QHBoxLayout(this); // 水平布局

    // 创建两个面板用于显示在界面左右两侧
    leftPanel = new QWidget(this);
    rightPanel = new QWidget(this);

    // 给左右侧 Widget 添加事件过滤器
    leftPanel->installEventFilter(this);
    rightPanel->installEventFilter(this);

    auto leftLayout = new QVBoxLayout();  // 垂直布局用于左侧
    auto rightLayout = new QVBoxLayout(); // 垂直布局用于右侧

    // 在两个面板中各添加一个标签
    auto leftLabel = new QLabel("闯关模式", this);
    auto rightLabel = new QLabel("休闲模式", this);

    // 设置标签居中显示
    leftLabel->setAlignment(Qt::AlignCenter);
    rightLabel->setAlignment(Qt::AlignCenter);

    // 把标签添加到各自布局中
    leftLayout->addWidget(leftLabel);
    rightLayout->addWidget(rightLabel);

    // 应用布局到面板
    leftPanel->setLayout(leftLayout);
    rightPanel->setLayout(rightLayout);

    // 添加面板到主布局
    mainLayout->addWidget(leftPanel);
    mainLayout->addWidget(rightPanel);

    this->show();
}

bool Beginwidget::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::Enter)
    {
        // 当鼠标进入 Widget 时改变背景色
        QWidget *widget = static_cast<QWidget *>(watched);
        if (widget)
            widget->setStyleSheet("background-color: lightgray;");
    }
    else if (event->type() == QEvent::Leave)
    {
        // 当鼠标离开 Widget 时恢复背景色
        QWidget *widget = static_cast<QWidget *>(watched);
        if (widget)
            // widget->setStyleSheet("background-image:url(:/images/1);");
            widget->setStyleSheet("background-color: white;");
    }
    else if (event->type() == QEvent::MouseButtonPress)
    {
        // 检测鼠标按键按下事件
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
        if (mouseEvent->button() == Qt::LeftButton)
        {
            QWidget *widget = static_cast<QWidget *>(watched);
            if (widget)
            {
                if (watched == leftPanel)
                {
                    // 在左侧布局中添加三个按钮
                    QVBoxLayout *leftLayout = new QVBoxLayout(widget);
                    QPushButton *ptn1 = new QPushButton("简单", widget);
                    QPushButton *ptn2 = new QPushButton("普通", widget);
                    QPushButton *ptn3 = new QPushButton("困难", widget);
                    leftLayout->addWidget(ptn1);
                    leftLayout->addWidget(ptn2);
                    leftLayout->addWidget(ptn3);
                    // 将原有的布局从QWidget中删除
                    delete widget->layout();
                    widget->setLayout(leftLayout);
                    box = nullptr;
                    // 使能
                    connect(ptn1, &QPushButton::clicked, [&]()
                            { box = new LeftMessageBox(Difficulty::briefness, this);                    
                            if (box->exec() == QDialog::Accepted)
                            {
                                emit beginGame();
                            } });
                    connect(ptn2, &QPushButton::clicked, [&]()
                            { box = new LeftMessageBox(Difficulty::ordinary, this);                    
                            if (box->exec() == QDialog::Accepted)
                            {
                                emit beginGame();
                            } });
                    connect(ptn3, &QPushButton::clicked, [&]()
                            { box = new LeftMessageBox(Difficulty::hard, this);                    
                            if (box->exec() == QDialog::Accepted)
                            {
                                emit beginGame();
                            } });
                    rightPanel->removeEventFilter(this);
                }
                else if (watched == rightPanel)
                {
                    dia = new RightDialog(this);
                    if (dia->exec() == QDialog::Accepted)
                    {
                        emit beginGame();
                    }
                    leftPanel->removeEventFilter(this);
                }
            }
        }
    }

    return QDialog::eventFilter(watched, event);
}

LeftMessageBox::LeftMessageBox(Difficulty diff, QWidget *parent) : QMessageBox(parent)
{
    initVariable();
    setWindowTitle("闯关模式");
    switch (diff)
    {
    case Difficulty::briefness:
    {
        setText("你已选择了[简单]模式，让我们开始游戏吧！");
        MacroDf::mVarable->m_diff = Difficulty::briefness;
        MacroDf::mVarable->m_lists->at(0)->time = MacroDf::mVarable->m_lists->at(0)->time * 2;
        MacroDf::mVarable->m_lists->at(1)->time = MacroDf::mVarable->m_lists->at(1)->time * 2;
        MacroDf::mVarable->m_lists->at(2)->time = MacroDf::mVarable->m_lists->at(2)->time * 2;
        MacroDf::mVarable->m_lists->at(3)->time = MacroDf::mVarable->m_lists->at(3)->time * 2;
    }
    break;
    case Difficulty::hard:
    {
        setText("你已选择了[困难]模式，让我们开始游戏吧！");
        MacroDf::mVarable->m_diff = Difficulty::hard;
        MacroDf::mVarable->m_lists->at(0)->time = MacroDf::mVarable->m_lists->at(0)->time / 2;
        MacroDf::mVarable->m_lists->at(1)->time = MacroDf::mVarable->m_lists->at(1)->time / 2;
        MacroDf::mVarable->m_lists->at(2)->time = MacroDf::mVarable->m_lists->at(2)->time / 2;
        MacroDf::mVarable->m_lists->at(3)->time = MacroDf::mVarable->m_lists->at(3)->time / 2;
    }
    break;
    case Difficulty::ordinary:
    {
        setText("你已选择了[普通]模式，让我们开始游戏吧！");
        MacroDf::mVarable->m_diff = Difficulty::ordinary;
    }
    break;
    default:
        break;
    }
    QPushButton *customButton = addButton("开始游戏", QMessageBox::AcceptRole);
    connect(customButton, &QPushButton::clicked, [=]()
            { parent->close();                              
            MacroDf::mVarable->m_model = Model::customsPass; });
    connect(customButton, &QPushButton::clicked, this, &QMessageBox::accept);
}

LeftMessageBox::~LeftMessageBox()
{
}

void LeftMessageBox::initVariable()
{
    QList<int> numbers;
    qsrand(static_cast<uint>(QTime::currentTime().msec())); // 初始化随机种子
    for (int i = 0; numbers.size() < 4; ++i)
    {
        int randNumber = qrand() % 12 + 1; // 生成[1, 12]范围内的随机数,图片的别名设置为这些
        if (!numbers.contains(randNumber))
        {
            numbers.append(randNumber);
        }
    }
    MacroDf::mVarable->m_lists = new QList<ModelRelaxation *>();
    auto data1 = new ModelRelaxation;
    data1->str = QString("2x2");
    data1->pixmap = QPixmap(QString(":/images/%1").arg(numbers.at(0)));
    data1->time = 30;
    auto data2 = new ModelRelaxation;
    data2->str = QString("3x3");
    data2->pixmap = QPixmap(QString(":/images/%1").arg(numbers.at(1)));
    data2->time = 60;
    auto data3 = new ModelRelaxation;
    data3->str = QString("4x4");
    data3->pixmap = QPixmap(QString(":/images/%1").arg(numbers.at(2)));
    data3->time = 120;
    auto data4 = new ModelRelaxation;
    data4->str = QString("5x5");
    data4->pixmap = QPixmap(QString(":/images/%1").arg(numbers.at(3)));
    data4->time = 240;

    MacroDf::mVarable->m_lists->push_back(data1);
    MacroDf::mVarable->m_lists->push_back(data2);
    MacroDf::mVarable->m_lists->push_back(data3);
    MacroDf::mVarable->m_lists->push_back(data4);
}

RightDialog::RightDialog(QWidget *parent) : QDialog(parent)
{
    MacroDf::mVarable->m_relax = new ModelRelaxation;
    setWindowTitle("休闲模式");
    // 设置HBoxLayout作为我们的布局容器
    QGridLayout *layout = new QGridLayout(this);
    QLabel *label = new QLabel("选择难度:");
    QComboBox *comboBox = new QComboBox();
    comboBox->addItems(comboxlists);
    QPushButton *button = new QPushButton("选择图片");

    // 创建一个QLabel用于存放图片，并设置其固定宽高
    imageLabel = new QLabel();
    QPixmap pixmap(":/images/2");
    imageLabel->setPixmap(pixmap.scaled(40, 40, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    // 开始按钮
    QPushButton *btn = new QPushButton("开始游戏", this);
    QSpacerItem *item = new QSpacerItem(10, 20);

    // 添加组件到布局中
    layout->addWidget(label, 0, 0);
    layout->addWidget(comboBox, 0, 1);
    layout->addItem(item, 0, 2);
    layout->addWidget(button, 0, 3);
    layout->addWidget(imageLabel, 0, 4);
    layout->addWidget(btn, 1, 4);
    // 设置窗口布局
    setLayout(layout);
    // 要在显示前就链接
    connect(btn, &QPushButton::clicked, [=]()
            { 
                close(); 
                parent->close();
                MacroDf::mVarable->m_model = Model::relaxation; });
    connect(btn, &QPushButton::clicked, this, &RightDialog::accept);
    connect(button, &QPushButton::clicked, this, &RightDialog::onPushbutton);
    connect(comboBox, &QComboBox::currentTextChanged, this, &RightDialog::onComboBox);
}

void RightDialog::onPushbutton()
{
    const QString filePath = QFileDialog::getOpenFileName(&QWidget(), "打开图片", "", "JPG(*.jpg)");
    QPixmap pixmap(filePath);
    imageLabel->setPixmap(pixmap.scaled(40, 40, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    MacroDf::mVarable->m_relax->pixmap = pixmap;
}

void RightDialog::onComboBox(const QString &text)
{
    MacroDf::mVarable->m_relax->str = text;
}
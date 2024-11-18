#include "stackedwidget.h"

#include <QSplitter>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>

MainWindowS::MainWindowS(QWidget *parent) : QMainWindow(parent)
{
    initUI();
    connect(listWidget, &QListWidget::currentRowChanged, this, &MainWindowS::onCurrentRowChanged);
}

MainWindowS::~MainWindowS()
{
}

void MainWindowS::onCurrentRowChanged(int row)
{
    stackWidget->setCurrentIndex(row);
}

void MainWindowS::initUI()
{
    resize(600, 480);
    QWidget *centralWidget = new QWidget(this);
    QSplitter *splitter = new QSplitter(centralWidget);
    splitter->setGeometry(QRect(5, 5, 600, 450));
    splitter->setFrameShape(QFrame::Box);
    splitter->setFrameShadow(QFrame::Plain);
    splitter->setLineWidth(1);
    splitter->setMidLineWidth(3);
    splitter->setOrientation(Qt::Horizontal);
    splitter->setOpaqueResize(true);

    listWidget = new QListWidget(splitter);
    listWidget->addItems(QStringList() << "第一页"
                                       << "第二页"
                                       << "第三页");

    splitter->addWidget(listWidget);
    stackWidget = new QStackedWidget(splitter);
    stackWidget->setCurrentIndex(0);

    QWidget *tab = new QWidget(stackWidget);
    QGridLayout *layout = new QGridLayout(tab);
    QLabel *tab_label = new QLabel(tab);
    tab_label->setText("我是第一页");
    layout->addWidget(tab_label, 0, 0);
    QLabel *label = new QLabel(tab);
    label->setScaledContents(true);
    label->setPixmap(QPixmap("../../1.jpg"));
    layout->addWidget(label, 1, 0);
    stackWidget->addWidget(tab);

    QWidget *tab2 = new QWidget(stackWidget);
    QGridLayout *layout2 = new QGridLayout(tab2);
    QLabel *tab_label2 = new QLabel(tab2);
    tab_label2->setText("我是第二页");
    layout2->addWidget(tab_label2, 0, 0);
    QLabel *label2 = new QLabel(tab2);
    label2->setScaledContents(true);
    label2->setPixmap(QPixmap("../../2.jpg"));
    layout2->addWidget(label2, 1, 0);
    stackWidget->addWidget(tab2);

    QWidget *tab3 = new QWidget(stackWidget);
    QGridLayout *layout3 = new QGridLayout(tab3);
    QLabel *tab_label3 = new QLabel(tab3);
    tab_label3->setText("我是第三页");
    layout3->addWidget(tab_label3, 0, 0);
    QLabel *label3 = new QLabel(tab3);
    label3->setScaledContents(true);
    label3->setPixmap(QPixmap("../../3.jpg"));
    layout3->addWidget(label3, 1, 0);
    stackWidget->addWidget(tab3);

    splitter->addWidget(stackWidget);
    splitter->setSizes(QList<int>() << 1 << 5);
    setCentralWidget(centralWidget);
}

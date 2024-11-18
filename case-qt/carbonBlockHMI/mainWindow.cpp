#include "mainWindow.h"
#include "tarpage.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    resize(800, 600);
    initUI();
}

MainWindow::~MainWindow()
{
}

void MainWindow::initUI()
{
    TarPage *m_tarPage = new TarPage(this);

    setCentralWidget(m_tarPage);
}

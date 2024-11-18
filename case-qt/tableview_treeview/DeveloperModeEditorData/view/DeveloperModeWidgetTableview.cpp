#include "DeveloperModeWidgetTableview.h"

#include <QFileDialog>
#include <QHeaderView>
#include <QMenu>
#include <QPainter>
#include <QSvgRenderer>

TableView::TableView(TableModel* model, QWidget* parent) : QTableView(parent), m_model(model)
{
    // 设置允许自定义上下文菜单
    setContextMenuPolicy(Qt::CustomContextMenu);
    // 设置委托
    MyDelegate* delegate = new MyDelegate(this);
    setItemDelegate(delegate);
    // 视图设置模型
    setModel(model);
    // 表头设置
    QHeaderView* horizontalHeader = this->horizontalHeader();
    // 允许手动调整大小
    horizontalHeader->setSectionResizeMode(QHeaderView::Interactive);
    // 设置最小宽度
    horizontalHeader->setMinimumSectionSize(180);
    // 最后一列自动填充列宽
    horizontalHeader->setStretchLastSection(true);

    connect(this, &QWidget::customContextMenuRequested, this, &TableView::showContextMenu);
}

void TableView::useImageMenu(QModelIndex index, const QString& strs)
{
    QMenu* menu = new QMenu(this);
    QAction* input = new QAction("导入图片", this);  // 导入图片
    QAction* delect = new QAction("删除图片", this); // 删除图片
    menu->addAction(input);
    menu->addAction(delect);
    QAction* act = menu->exec(QCursor::pos());
    if (act && act->text() == "导入图片") {
        inputImage(index, strs);
    } else if (act && act->text() == "删除图片") {
        if (KMessageBox::warning(this, "警告", "删除后不可恢复，是否确认删除？",
                                 KMessageBox::Yes | KMessageBox::No) // 删除后不可恢复，是否确认删除？
            == KMessageBox::Yes) {
            if (this->model())
                this->model()->setData(index, QVariant(), Qt::EditRole);
        }
    }
}

void TableView::inputImage(QModelIndex index, const QString& strs)
{
    if (Developer::SVGImageLists.contains(strs)) {
        QString fileName =
                QFileDialog::getOpenFileName(this, QObject::tr("select image"), "", tr("Image Files (*.svg)"));
        if (!fileName.isEmpty()) {
            QFile file(fileName);
            file.open(QFile::ReadOnly);
            auto svgData = file.readAll();
            if (this->model())
                this->model()->setData(index, svgData, Qt::EditRole);
        }
    }
    if (Developer::PNGImageLists.contains(strs)) {
        QString filePath = QFileDialog::getOpenFileName(this, QObject::tr("select image"), "", "PNG(*.png)");
        if (!filePath.isEmpty()) {
            QImage image;
            QFile file(filePath);
            file.open(QFile::ReadOnly);
            image.loadFromData(file.readAll());
            // 将QImage存储为QVariant
            QVariant variant;
            variant.setValue(image);
            if (this->model())
                this->model()->setData(index, variant, Qt::EditRole);
        }
    }
}

TableView::~TableView() { }

void TableView::showContextMenu(const QPoint& pos)
{
    QModelIndex index = indexAt(pos);
    if (index.isValid()) {
        if (index.column() == TableModel::Value) {
            QString keyname = Kcc::BlockDefinition::RoleDataDefinition::desc2DataDef(
                    index.sibling(index.row(), TableModel::Key).data().toString());
            if (Developer::PNGImageLists.contains(keyname) || Developer::SVGImageLists.contains(keyname)) {
                useImageMenu(index, keyname);
                return;
            }
        }
    }

    QMenu contextMenu(tr("菜单"), this);

    QAction addAction(tr("添加键值"), this);
    QAction removeAction(tr("删除"), this);

    connect(&addAction, &QAction::triggered, this, [=] {
        // 添加一行
        static_cast<TableModel*>(model())->addRow(Developer::newKey, Developer::newValue);
    });

    connect(&removeAction, &QAction::triggered, this, [=] {
        if (index.isValid()) {
            // 删除指定的行
            static_cast<TableModel*>(model())->removeRow(index.row());
        }
    });

    // 只有在存在有效行时添加删除选项
    if (index.isValid()) {
        contextMenu.addAction(&removeAction);
    }
    contextMenu.addAction(&addAction);

    // 显示菜单
    contextMenu.exec(mapToGlobal(pos));
}
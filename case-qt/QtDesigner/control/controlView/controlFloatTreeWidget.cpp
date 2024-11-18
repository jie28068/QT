#include "controlFloatTreeWidget.h"
#include "controlTreeItem.h"
#include "controlTreeModel.h"

#include <QTreeView>
#include <QHeaderView>
#include <QLayout>
#include <QDebug>
#include <QPushButton>

ControlFloatTreeWidget::ControlFloatTreeWidget(QWidget *parent) : QWidget(parent)

{
    treeView = new QTreeView(this);
    treeView->setSelectionBehavior(QTreeView::SelectRows);  // 一次选中整行
    treeView->setSelectionMode(QTreeView::SingleSelection); // 单选，配合上面的整行就是一次选单行
    treeView->setFocusPolicy(Qt::NoFocus);                  // 去掉鼠标移到单元格上时的虚线框
    treeView->header()->setStretchLastSection(true);        // 最后一列自适应宽度

    QPushButton *btn = new QPushButton("提升", this);
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(btn);
    layout->addWidget(treeView);
    setLayout(layout);
    QVector<GlobalDefinition::Province *> proList = initData();
    setModel(proList);

    connect(btn, &QPushButton::clicked, this, &ControlFloatTreeWidget::pClicked);
}

ControlFloatTreeWidget::~ControlFloatTreeWidget()
{
}

QVector<GlobalDefinition::Province *> ControlFloatTreeWidget::initData()
{
    // 初始化数据，5个省，每个省5人
    QVector<GlobalDefinition::Province *> proList;
    int provinceCount = 5;
    int personCount = 5;
    for (int i = 0; i < provinceCount; i++)
    {
        GlobalDefinition::Province *pro = new GlobalDefinition::Province();
        pro->name = QString("Province%1").arg(i);
        for (int j = 0; j < personCount; j++)
        {
            GlobalDefinition::Person *per = new GlobalDefinition::Person();
            per->name = QString("name%1").arg(j);
            per->sex = "man";
            per->age = 25;
            per->phone = "123456789";
            pro->people.append(per);
        }
        proList.append(pro);
    }
    return proList;
}

void ControlFloatTreeWidget::setModel(const QVector<GlobalDefinition::Province *> &proList)
{
    QStringList headers;
    headers << QString("名称/姓名")
            << QString("性别")
            << QString("年龄")
            << QString("电话");

    FloatTreeModel *model = new FloatTreeModel(headers, treeView);
    FloatTreeItem *root = model->root();
    foreach (auto pro, proList)
    {
        FloatTreeItem *province = new FloatTreeItem(root);
        province->setPtr(pro);                      // 保存数据指针
        province->setType(FloatTreeItem::PROVINCE); // 设置节点类型为PROVINCE
        root->addChild(province);

        foreach (auto per, pro->people)
        {
            FloatTreeItem *person = new FloatTreeItem(province);
            person->setPtr(per);                    // 保存数据指针
            person->setType(FloatTreeItem::PERSON); // 设置节点类型为PERSON
            province->addChild(person);
        }
    }

    treeView->setModel(model);
    connect(model, &QAbstractItemModel::dataChanged,
            this, &ControlFloatTreeWidget::onTreeDataChanged);
}

void ControlFloatTreeWidget::onTreeDataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight)
{
    FloatTreeItem *item = static_cast<FloatTreeItem *>(topLeft.internalPointer());
    if (item->checkable(topLeft.column()))
    {
        QString name = item->data(GlobalDefinition::COLUMN_NAME).toString();
        if (item->isChecked())
        {
            qDebug() << "checked:" << name;
        }
        else
        {
            qDebug() << "unchecked:" << name;
        }
    }
}
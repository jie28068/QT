
#include "controlTool.h"
#include "customDockWidget.h"
#include "mainWindow.h"
#include "globalDefinition.h"
#include "controlDialogTreeView.h"
#include "controPropertyWidget.h"
#include "ControlManyToOne.h"
#include "ControlOneToMany.h"
#include "qtvariantproperty.h"
#include "qttreepropertybrowser.h"
#include "controlFloatTreeWidget.h"

#include <QStatusBar>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    initUI();
    setFixedSize(1200, 600);
}

MainWindow::~MainWindow()
{
}

void MainWindow::initUI()
{
    ControlTools *w = new ControlTools(this);
    w->addPage(GlobalDefinition::controlParameters);
    // w->addPage(GlobalDefinition::controlContainer);
    // w->addPage(GlobalDefinition::controlDisplay);
    // w->addPage(GlobalDefinition::controlOperation);

    // // 顶层菜单
    // QToolBar *toolBar = new QToolBar(this);
    // toolBar->addWidget(new QPushButton(QString::fromUtf8("预览")));
    // this->addToolBar(Qt::TopToolBarArea, toolBar);
    // 左停靠窗口
    m_dockListView = new CustomDockWidget(QString::fromUtf8("控件"), this);
    m_dockListView->setWidget(w);
    this->addDockWidget(Qt::LeftDockWidgetArea, m_dockListView);

    m_dockConstraint = new CustomDockWidget(QString::fromUtf8("约束"), this);
    m_dockConstraint->setWidget(new QTextEdit(QString::fromUtf8("约束窗口")));
    this->addDockWidget(Qt::LeftDockWidgetArea, m_dockConstraint);
    m_dockConstraint->setVisible(false);

    // 中间
    QWidget *centerWidget = new QWidget(this);
    QTabWidget *tabWidget = new QTabWidget(centerWidget);
    // 创建页面
    QWidget *page1 = new QWidget(tabWidget);
    QWidget *page2 = new QWidget(tabWidget);
    // 为每个页面添加一些内容
    treeView = new ControlDialogTreeView(page1);
    QVBoxLayout *page1Layout = new QVBoxLayout(page1);
    page1Layout->addWidget(treeView);
    page1->setLayout(page1Layout);

    QLabel *label2 = new QLabel("33333", page2);
    // 将页面添加到QTabWidget
    tabWidget->addTab(page1, "参数与对话框");
    tabWidget->addTab(page2, "图标");
    QHBoxLayout *layout = new QHBoxLayout();
    layout->addWidget(tabWidget);
    centerWidget->setLayout(layout);
    this->setCentralWidget(centerWidget);
    // end
    // 右停靠窗口
    m_dockProperty = new CustomDockWidget(QString::fromUtf8("属性编辑器"), this);
    variantManager = new QtVariantPropertyManager(this);
    QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory(this);
    propertyEditor = new QtTreePropertyBrowser(m_dockProperty);
    propertyEditor->setFactoryForManager(variantManager, variantFactory);
    m_dockProperty->setWidget(propertyEditor);
    this->addDockWidget(Qt::RightDockWidgetArea, m_dockProperty);
    // end
    // 下方停靠窗口
    m_dockTreeView = new CustomDockWidget(QString::fromUtf8("控件树"), this);
    this->addDockWidget(Qt::BottomDockWidgetArea, m_dockTreeView);
    auto controlFloatTreeWidget = new ControlFloatTreeWidget(m_dockTreeView);
    m_dockTreeView->setVisible(false);
    m_dockTreeView->setWidget(controlFloatTreeWidget);
    // end
    connect(variantManager, &QtVariantPropertyManager::valueChanged,
            this, &MainWindow::valueChanged);
    connect(tabWidget, &QTabWidget::currentChanged, this, [=](int index)
            {
        if (index == 0)
        {
            m_dockListView->setVisible(true);
            m_dockConstraint->setVisible(false);
            m_dockProperty->setVisible(true);
        }
        else
        {
            m_dockListView->setVisible(false);
            m_dockConstraint->setVisible(true);
            m_dockProperty->setVisible(false);
        } });

    connect(treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updatePositions);
    connect(treeView, &QAbstractItemView::clicked, this, &MainWindow::itemClicked);
    connect(w, &ControlTools::clickedControlTools, this, &MainWindow::clickedControlView);
    connect(w, &ControlTools::clickedControlTools, this, &MainWindow::clickedControlTreeView);
    connect(controlFloatTreeWidget, &ControlFloatTreeWidget::pClicked, [&]()
            { m_dockTreeView->setVisible(false); });
}

void MainWindow::updatePositions()
{
    const int row = treeView->selectionModel()->currentIndex().row();
    const int column = treeView->selectionModel()->currentIndex().column();
    if (treeView->selectionModel()->currentIndex().parent().isValid())
        statusBar()->showMessage(tr("Position: (%1,%2)").arg(row).arg(column));
    else
        statusBar()->showMessage(tr("Position: (%1,%2) in top level").arg(row).arg(column));
}

void MainWindow::addProperty(QtVariantProperty *property, const QString &id)
{
    propertyToId[property] = id;
    idToProperty[id] = property;
    QtBrowserItem *item = propertyEditor->addProperty(property);
}

void MainWindow::itemClicked(const QModelIndex &index)
{
    QMap<QtProperty *, QString>::ConstIterator itProp = propertyToId.constBegin();
    while (itProp != propertyToId.constEnd())
    {
        delete itProp.key();
        itProp++;
    }
    propertyToId.clear();
    idToProperty.clear();

    auto data = index.siblingAtColumn(0).data(GlobalDefinition::controlItemData);

    QtVariantProperty *property;
    auto item = data.value<ControlBase *>();
    currentItem = item;
    if (item)
    {
        if (item->rtti() == ControlBase::ManyToOneControl)
        {
            auto p = dynamic_cast<ControlManyToOne *>(item);
            property = variantManager->addProperty(QVariant::String, "名称");
            property->setValue(p->controlName());
            addProperty(property, "名称");

            property = variantManager->addProperty(QVariant::String, "变量值");
            property->setValue(p->controlValue());
            addProperty(property, "变量值");

            property = variantManager->addProperty(QVariant::String, "显示名称");
            property->setValue(p->controlDisplayName());
            addProperty(property, "显示名称");

            property = variantManager->addProperty(QVariant::Bool, "端口可见");
            property->setValue(p->controlVisiblePort());
            addProperty(property, "端口可见");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "可见类型");
            QStringList enumNamesV;
            enumNamesV << "不可见" << "可见不可编辑" << "可见可编辑";
            property->setAttribute(QLatin1String("enumNames"), enumNamesV);
            property->setValue(p->controlVisibleType());
            addProperty(property, "可见类型");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "限幅类型");
            QStringList enumNames;
            enumNames << "无限幅" << "上限幅" << "下限幅";
            property->setAttribute(QLatin1String("enumNames"), enumNames);
            property->setValue(p->controlLimitingType());
            addProperty(property, "限幅类型");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "控件类型");
            QStringList enumNamesT;
            enumNamesT << "复数" << "字体" << "整型" << "浮点型" << "日期" << "颜色" << "下拉框" << "文本框" << "勾选框" << "写文件" << "读文件" << "整型数组" << "浮点型数组" << "数学表达式" << "可输入下拉框" << "带预设项下拉框";
            property->setAttribute(QLatin1String("enumNames"), enumNamesT);
            property->setValue(p->controlType());
            addProperty(property, "控件类型");

            property = variantManager->addProperty(QVariant::String, "控件值");
            property->setValue(p->controlTypeValue());
            addProperty(property, "控件值");

            property = variantManager->addProperty(QVariant::String, "变量说明");
            property->setValue(p->controlPrompt());
            addProperty(property, "变量说明");
        }
        else if (item->rtti() == ControlBase::OneToManyControl)
        {
            auto p = dynamic_cast<ControlOneToMany *>(item);
            property = variantManager->addProperty(QVariant::String, "名称");
            property->setValue(p->controlName());
            addProperty(property, "名称");

            property = variantManager->addProperty(QVariant::String, "变量值");
            property->setValue(p->controlValue());
            addProperty(property, "变量值");

            property = variantManager->addProperty(QVariant::String, "显示名称");
            property->setValue(p->controlDisplayName());
            addProperty(property, "显示名称");

            property = variantManager->addProperty(QVariant::Bool, "端口可见");
            property->setValue(p->controlVisiblePort());
            addProperty(property, "端口可见");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "可见类型");
            QStringList enumNamesV;
            enumNamesV << "不可见" << "可见不可编辑" << "可见可编辑";
            property->setAttribute(QLatin1String("enumNames"), enumNamesV);
            property->setValue(p->controlVisibleType());
            addProperty(property, "可见类型");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "限幅类型");
            QStringList enumNames;
            enumNames << "无限幅" << "上限幅" << "下限幅";
            property->setAttribute(QLatin1String("enumNames"), enumNames);
            property->setValue(p->controlLimitingType());
            addProperty(property, "限幅类型");

            property = variantManager->addProperty(QtVariantPropertyManager::enumTypeId(), "控件类型");
            QStringList enumNamesT;
            enumNamesT << "复数" << "字体" << "整型" << "浮点型" << "日期" << "颜色" << "下拉框" << "文本框" << "勾选框" << "写文件" << "读文件" << "整型数组" << "浮点型数组" << "数学表达式" << "可输入下拉框" << "带预设项下拉框";
            property->setAttribute(QLatin1String("enumNames"), enumNamesT);
            property->setValue(p->controlType());
            addProperty(property, "控件类型");

            property = variantManager->addProperty(QVariant::String, "控件值");
            property->setValue(p->controlTypeValue());
            addProperty(property, "控件值");

            property = variantManager->addProperty(QVariant::String, "变量说明");
            property->setValue(p->controlPrompt());
            addProperty(property, "变量说明");
        }
    }
}

void MainWindow::valueChanged(QtProperty *property, const QVariant &value)
{
    if (!propertyToId.contains(property))
    {
        return;
    }

    QString id = propertyToId[property];

    if (currentItem->rtti() == ControlBase::ManyToOneControl)
    {
        auto p = dynamic_cast<ControlManyToOne *>(currentItem);

        if (id == "名称")
        {
            p->setControlName(value.toString());
        }
        else if (id == "变量值")
        {
            p->setControlValue(value.toString());
        }
        else if (id == "显示名称")
        {
            p->setControlDisplayName(value.toString());
        }
        else if (id == "端口可见")
        {
            p->setControlVisiblePort(value.toBool());
        }
        else if (id == "可见类型")
        {
            p->setControlVisibleType(static_cast<ControlManyToOne::ControlVisibleType>(value.toInt()));
        }
        else if (id == "限幅类型")
        {
            p->setControlLimitingType(static_cast<ControlManyToOne::ControlLimitingType>(value.toInt()));
        }
        else if (id == "控件类型")
        {
            p->setControlType(static_cast<ControlManyToOne::ControlType>(value.toInt()));
        }
        else if (id == "控件值")
        {
            p->setControlTypeValue(value.toString());
        }
        else if (id == "变量说明")
        {
            p->setControlPrompt(value.toString());
        }
    }
    else if (currentItem->rtti() == ControlBase::OneToManyControl)
    {
        auto p = dynamic_cast<ControlOneToMany *>(currentItem);
        if (id == "名称")
        {
            p->setControlName(value.toString());
        }
        else if (id == "变量值")
        {
            p->setControlValue(value.toString());
        }
        else if (id == "显示名称")
        {
            p->setControlDisplayName(value.toString());
        }
        else if (id == "端口可见")
        {
            p->setControlVisiblePort(value.toBool());
        }
        else if (id == "可见类型")
        {
            p->setControlVisibleType(static_cast<ControlOneToMany::ControlVisibleType>(value.toInt()));
        }
        else if (id == "限幅类型")
        {
            p->setControlLimitingType(static_cast<ControlOneToMany::ControlLimitingType>(value.toInt()));
        }
        else if (id == "控件类型")
        {
            p->setControlType(static_cast<ControlOneToMany::ControlType>(value.toInt()));
        }
        else if (id == "控件值")
        {
            p->setControlTypeValue(value.toString());
        }
        else if (id == "变量说明")
        {
            p->setControlPrompt(value.toString());
        }
    }
}

void MainWindow::clickedControlView(QMimeData *data)
{
    QByteArray encodedData;
    if (data->hasFormat(GlobalDefinition::controlMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlMimeType);
    }
    else if (data->hasFormat(GlobalDefinition::controlDialogMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlDialogMimeType);
    }
    treeView->insertChild(encodedData, QModelIndex());
}

void MainWindow::clickedControlTreeView(QMimeData *data)
{
    QByteArray encodedData;
    if (data->hasFormat(GlobalDefinition::controlMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlMimeType);
    }
    else if (data->hasFormat(GlobalDefinition::controlDialogMimeType))
    {
        encodedData = data->data(GlobalDefinition::controlDialogMimeType);
    }
    m_dockTreeView->setVisible(true);
}

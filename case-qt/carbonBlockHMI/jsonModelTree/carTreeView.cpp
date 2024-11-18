#include "carTreeView.h"
#include "QJsonModel.h"
#include "jsonDefines.h"
#include "tarTreeDelegate.h"

#include <QContextMenuEvent>
#include <QDebug>
#include <QHeaderView>
#include <QToolTip>
CarTreeView::CarTreeView(QWidget *parent) : QTreeView(parent)
{
    m_menu = new QMenu(this);
    m_model = new QJsonModel;
    TarTreeDelegate *delegate = new TarTreeDelegate(this);
    setModel(m_model);
    setItemDelegate(delegate);
    header()->resizeSection(0, 400);
    setMouseTracking(true);
    connect(this, &CarTreeView::entered, this, &CarTreeView::showTooltip);
    setAlternatingRowColors(true);
    setStyleSheet(
        "QMenu {"
        "background-color: #f0f0f0;" // 菜单背景颜色
        "border: 1px solid #999999;" // 菜单边框
        "}"
        "QMenu::item {"
        "color: #000000;" // 菜单项文本颜色
        "}"
        "QMenu::item:selected {"     // 选中项样式
        "background-color: #4a90d9;" // 选中项背景颜色
        "color: #ffffff;"            // 选中项文本颜色
        "}");
    setStyleSheet(JsonDefines::PAGE_TREE_QSS);

    connect(m_model, &QJsonModel::dataValueChanged, [=](const QString &key, const QVariant &value, const QVariant &oldValue)
            {
                if(value != oldValue){
            QString str=  QString("属性[%1]的值[%2] 变更为 值[%3]").arg(key, oldValue.toString(), value.toString());
            emit message(str);} });
}

void CarTreeView::setTarList(const QStringList &tarList)
{
    m_tarList = tarList;
}

QJsonModel *CarTreeView::getJsonModel()
{
    return m_model;
}

void CarTreeView::addTarItem(QJsonObject jsonObject, const QString &tarName)
{
    int rowCount = m_model->rowCount(QModelIndex());
    for (int row = 0; row < rowCount; ++row)
    {
        QModelIndex index = m_model->index(row, 0, QModelIndex());
        QString currentIndexName = m_model->data(index, Qt::DisplayRole).toString();
        if (currentIndexName == JsonDefines::TARLIST)
        {
            m_model->addArrayItem(index, jsonObject, tarName);
        }
    }
}
void CarTreeView::setEditable(bool editable)
{
    if (editable)
    {
        setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    }
    else
    {
        setEditTriggers(QAbstractItemView::NoEditTriggers);
    }
}
void CarTreeView::showTooltip(QModelIndex index)
{
    QToolTip::showText(QCursor::pos(), index.data().toString());
}

void CarTreeView::contextMenuEvent(QContextMenuEvent *event)
{
    if (editTriggers() == QAbstractItemView::NoEditTriggers)
    {
        event->ignore();
        return;
    }
    // 获取点击的项
    QModelIndex index = indexAt(event->pos());
    QJsonModel *model = qobject_cast<QJsonModel *>(this->model());

    if (index.isValid() && model)
    {
        if (isItemValidTarList(index))
        {
            m_menu->clear();
            QAction *action = m_menu->addAction("添加轨迹");
            QAction *actionD = m_menu->addAction("清空轨迹");
            QAction *selectedAction = m_menu->exec(event->globalPos());
            if (selectedAction == action)
            {
                emit addTar();
                emit message("添加轨迹");
            }
            else if (selectedAction == actionD)
            {
                model->removeAllItems(index);
                emit message("清空轨迹");
            }
        }

       else if (isItemValidTar(index))
        {
            m_menu->clear();
            QAction *action = m_menu->addAction("删除轨迹");
            QAction *selectedAction = m_menu->exec(event->globalPos());
            if (selectedAction == action)
            {
                model->removeArrayItem(index);
                emit message("删除轨迹");
            }
        }

        else if (isItemValidBowls(index))
        {
            m_menu->clear();
            QAction *action = m_menu->addAction("添加碳碗");
            QAction *actionD = m_menu->addAction("删除碳碗");
            QAction *selectedAction = m_menu->exec(event->globalPos());
            if (selectedAction == action)
            {
                QJsonObject newBowl = getCarbonBowlJsonObject();
                newBowl[JsonDefines::WORKNAME] = JsonDefines::BOWL + QString::number(model->getIndexChildCount(index) + 1);
                model->addArrayItem(index, newBowl, JsonDefines::BOWL);
                emit message("添加碳碗");
            }
            else if ((selectedAction == actionD))
            {
                model->removeLastItem(index);
                emit message("删除碳碗");
            }
        }
    }
}

bool CarTreeView::isItemValidTarList(const QModelIndex &index)
{
    if (index.column() == 0)
    {
        QJsonModel *model = qobject_cast<QJsonModel *>(this->model());
        auto str = model->data(index, Qt::DisplayRole).toString();
        if (str == JsonDefines::TARLIST)
        {
            return true;
        }
    }
    return false;
}

bool CarTreeView::isItemValidTar(const QModelIndex &index)
{
    if (index.column() == 0)
    {
        QJsonModel *model = qobject_cast<QJsonModel *>(this->model());
        auto str = model->data(index, Qt::DisplayRole).toString();
        if (m_tarList.contains(str) && model->isTar(index))
        {
            return true;
        }
    }
    return false;
}

bool CarTreeView::isItemValidBowls(const QModelIndex &index)
{
    if (index.column() == 0)
    {
        QJsonModel *model = qobject_cast<QJsonModel *>(this->model());
        auto str = model->data(index, Qt::DisplayRole).toString();
        if (str == JsonDefines::BOWLS)
        {
            return true;
        }
    }
    return false;
}

QJsonObject CarTreeView::getCarbonBowlJsonObject()
{
    auto json = m_model->json();
    QJsonDocument doc = QJsonDocument::fromJson(json);
    QJsonObject jsonObj = doc.object();

    QJsonObject carbonInfo = jsonObj[JsonDefines::CARBONINFO].toObject();

    QJsonArray bowlsArray = carbonInfo[JsonDefines::BOWLS].toArray();

    QJsonObject firstBowl = bowlsArray[0].toObject();

    QJsonObject bowl = firstBowl[JsonDefines::BOWL].toObject();

    return bowl;
}

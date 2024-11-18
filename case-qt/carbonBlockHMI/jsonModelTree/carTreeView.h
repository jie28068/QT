#pragma once
#include <QTreeView>
#include <QObject>
#include <QMenu>

class QJsonModel;
class TreeViewProxyModel;
class CarTreeView : public QTreeView
{
    Q_OBJECT

public:
    explicit CarTreeView(QWidget *parent = nullptr);
    /// @brief 设置轨迹列表
    /// @param tarList
    void setTarList(const QStringList &tarList);

    QJsonModel *getJsonModel();
    /// @brief 添加轨迹
    /// @param jsonObject 数据
    /// @param tarName 名称
    void addTarItem(QJsonObject jsonObject, const QString &tarName);

    // 设置是否可编辑
    void setEditable(bool editable);

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void showTooltip(QModelIndex index);
signals:
    void addTar();
    void message(QString msg);

private:
    /// @brief 添加轨迹
    /// @param index
    /// @return
    bool isItemValidTarList(const QModelIndex &index);
    /// @brief 删除轨迹
    bool isItemValidTar(const QModelIndex &index);
    /// @brief 删除/添加碳碗
    /// @param index
    /// @return
    bool isItemValidBowls(const QModelIndex &index);
    /// @brief 获取碳碗数据
    /// @return
    QJsonObject getCarbonBowlJsonObject();

private:
    QStringList m_tarList; // 模版轨迹列表
    QMenu *m_menu;
    QJsonModel *m_model;
};
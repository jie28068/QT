#pragma once

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QJsonArray>

#include <QDebug>

class JsonDataManager : public QObject
{
    Q_OBJECT
public:
    explicit JsonDataManager(QObject *parent = nullptr);
    ~JsonDataManager();
    /// @brief 读取全局配置
    /// @return
    QJsonObject &loadGlobalProperties();
    /// @brief 轨迹配置
    /// @return
    QJsonObject &loadCarbonProperties();
    /// @brief 读取当前的json数据
    /// @return
    QJsonObject &loadCurrentJsonData();
    /// @brief 加载模版json
    void load(QByteArray data);
    /// @brief 加载链表json
    /// @param data
    void loadList(QByteArray data);
    /// @brief 保存
    /// @param data
    bool save(QJsonObject data);
    /// @brief 获取模版轨迹链表名称
    /// @return
    QStringList getCarbonList();
    /// @brief 获取轨迹链表名称
    /// @return
    QStringList getTarList();
signals:
    void updateJsonData();

private:
    QJsonObject m_jsonData;         // 原始数据
    QJsonObject m_globalProperties; // 全局属性
    QJsonObject m_carbonProperties; // 轨迹属性
    QJsonObject m_currentJsonData;  // 当前所有属性
    QStringList m_carbonList;       // 轨迹链表名称
};

#include "jsonDataManager.h"
#include "jsonDefines.h"
JsonDataManager::JsonDataManager(QObject *parent) : QObject(parent)
{
}

JsonDataManager::~JsonDataManager()
{
}

QJsonObject &JsonDataManager::loadGlobalProperties()
{
    return m_globalProperties;
}

QJsonObject &JsonDataManager::loadCarbonProperties()
{
    return m_carbonProperties;
}

QJsonObject &JsonDataManager::loadCurrentJsonData()
{
    return m_currentJsonData;
}

void JsonDataManager::loadList(QByteArray jsonData)
{
    m_carbonList.clear();
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
    m_jsonData = jsonDoc.object();
    QStringList keys = m_jsonData.keys();
    for (const QString &key : keys)
    {
        m_carbonList.append(key);
    }
    /// 初始化
    m_currentJsonData = m_jsonData;
}

void JsonDataManager::load(QByteArray jsonData)
{
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
    m_jsonData = jsonDoc.object();

    for (auto it = m_jsonData.begin(); it != m_jsonData.end(); ++it)
    {
        QJsonObject item = it.value().toObject();
        if (item[JsonDefines::TRACKTYPE].toString() == JsonDefines::TRACKTYPEDATA)
        {
            m_carbonProperties[it.key()] = item;
        }
        else
        {
            m_globalProperties[it.key()] = item;
        }
    }

    /// 初始化
    m_currentJsonData = m_globalProperties;

    QJsonArray traList;
    m_currentJsonData[JsonDefines::TARLIST] = traList;
}

bool JsonDataManager::save(QJsonObject data)
{
    QJsonDocument jsonDoc(data);
    QByteArray jsonData = jsonDoc.toJson();

    QFile file("/home/ray/myGitproject/my-case/linux/tar/carbon_block_clean/config/save.json");
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "Error: Could not open file for writing.";
        return false;
    }

    file.write(jsonData);
    file.close();
    return true;
}

QStringList JsonDataManager::getCarbonList()
{
    QStringList list;
    for (auto key : m_carbonProperties.keys())
    {
        list.append(key);
    }
    return list;
}

QStringList JsonDataManager::getTarList()
{
    return m_carbonList;
}

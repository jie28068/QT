#pragma once

#include <QAbstractItemModel>
#include <QIcon>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "QUtf8.h"

class QJsonModel;
class QJsonItem;

class QJsonTreeItem
{
public:
  QJsonTreeItem(QJsonTreeItem *parent = nullptr);
  ~QJsonTreeItem();
  void appendChild(QJsonTreeItem *item);
  QJsonTreeItem *child(int row);
  QJsonTreeItem *parent();
  int childCount() const;
  int row() const;
  void setKey(const QString &key);
  void setValue(const QVariant &value);
  void setType(const QJsonValue::Type &type);
  QString key() const;
  QVariant value() const;
  QJsonValue::Type type() const;

  static QJsonTreeItem *load(const QJsonValue &value,
                             const QStringList &exceptions = {},
                             QJsonTreeItem *parent = nullptr);

public:
  QList<QJsonTreeItem *> mChilds;

protected:
private:
  QString mKey;
  QVariant mValue;
  QJsonValue::Type mType;
  QJsonTreeItem *mParent = nullptr;
};

//---------------------------------------------------

class QJsonModel : public QAbstractItemModel
{
  Q_OBJECT
public:
  explicit QJsonModel(QObject *parent = nullptr);
  QJsonModel(const QString &fileName, QObject *parent = nullptr);
  QJsonModel(QIODevice *device, QObject *parent = nullptr);
  QJsonModel(const QByteArray &json, QObject *parent = nullptr);
  ~QJsonModel();
  bool load(const QString &fileName);
  bool load(QIODevice *device);
  bool loadJson(const QByteArray &json);
  QVariant data(const QModelIndex &index, int role) const override;
  bool setData(const QModelIndex &index, const QVariant &value,
               int role = Qt::EditRole) override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
  QModelIndex index(int row, int column,
                    const QModelIndex &parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex &index) const override;
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;
  int columnCount(const QModelIndex &parent = QModelIndex()) const override;
  Qt::ItemFlags flags(const QModelIndex &index) const override;
  QByteArray json(bool compact = false);
  QByteArray jsonToByte(QJsonValue jsonValue);
  void objectToJson(QJsonObject jsonObject, QByteArray &json, int indent,
                    bool compact);
  void arrayToJson(QJsonArray jsonArray, QByteArray &json, int indent,
                   bool compact);
  void arrayContentToJson(QJsonArray jsonArray, QByteArray &json, int indent,
                          bool compact);
  void objectContentToJson(QJsonObject jsonObject, QByteArray &json, int indent,
                           bool compact);
  void valueToJson(QJsonValue jsonValue, QByteArray &json, int indent,
                   bool compact);
  void addException(const QStringList &exceptions);

  // fixme
  //***通用***//
  /// 清除所有数据
  void cleraData();
  /// @brief 删除指定行
  /// @param index
  void removeItem(const QModelIndex &index);
  /// 删除最后一项
  void removeLastItem(const QModelIndex &index);
  /// @brief 返回当前项的子项数量
  int getIndexChildCount(const QModelIndex &index);
  /// @brief 删除所有子项
  /// @param index
  void removeAllItems(const QModelIndex &index);
  /// @brief 设置表头名称
  /// @param headers
  void setHearders(const QStringList &headers);

  //***特殊***//
  /// @brief 是否为轨迹
  /// @param index
  bool isTar(const QModelIndex &index);
  /// 删除数组节点
  void removeArrayItem(const QModelIndex &index);
  /// @brief 添加数组子项
  void addArrayItem(const QModelIndex &index, QJsonObject jsonObject, const QString &key);

signals:
  /// @brief 数据值改变
  void dataValueChanged(const QString &key, const QVariant &value, const QVariant &oldValue);

private:
  QJsonValue genJson(QJsonTreeItem *) const;
  QJsonTreeItem *mRootItem = nullptr;
  QStringList mHeaders;
  QStringList mExceptions;
};
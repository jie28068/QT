#ifndef CONTROLTOOLMODEL_H
#define CONTROLTOOLMODEL_H

#include <QApplication>
#include <QListView>
#include <QAbstractListModel>
#include <QIcon>

struct ModelData
{
    ModelData(QString name, bool isChecked, QString icon)
        : m_name(name), m_isChecked(isChecked), m_icon(icon)
    {
    }
    QString m_name;
    QString m_icon;
    bool m_isChecked;
};

class IconTextModel : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit IconTextModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    void addData(const QString &text, const QString &icon, bool isChecked);

private:
    QList<ModelData> m_data;
};

#endif
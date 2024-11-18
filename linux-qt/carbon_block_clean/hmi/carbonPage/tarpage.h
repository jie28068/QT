#ifndef TARPAGE_H
#define TARPAGE_H

#include <QWidget>
class QJsonModel;
class JsonDataManager;
class HostComputeClient;
namespace Ui
{
    class TarPage;
}

class TarPage : public QWidget
{
    Q_OBJECT

public:
    explicit TarPage(QSharedPointer<HostComputeClient> clint, QWidget *parent = nullptr);
    void initUI();
    void initConnect();
    ~TarPage();
    QByteArray getJsonData();

public slots:
    /// 保存json到文件
    void saveJsonFileData();
    /// 设置轨迹操作界面显隐
    void setCarbonWidgetShow();
    /// @brief 选择轨迹
    void switchTarItem(const QString &tarName);
    /// @brief 添加轨迹
    void addTarItem();
    /// @brief 炭块模版
    void carbonTemplate();
    /// @brief 获取碳块列表
    void carbonList();

private:
    /// @brief 消息提示
    void setMessageBox(const QString &msg, int type = 0);
    /// @brief  显示操作信息
    void setMessageOperation(const QString &msg);
    /// @brief 检测名称是否重复
    bool nameDetection(QByteArray &data);
    /// @brief 获取翻译
    void getTranslation();

private:
    Ui::TarPage *ui;
    QJsonModel *tar_model;
    JsonDataManager *dataManager;
    QSharedPointer<HostComputeClient> m_clint;
};

#endif // TARPAGE_H

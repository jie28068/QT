#ifndef ADJUSTPARAMPAGE_H
#define ADJUSTPARAMPAGE_H

#include <QWidget>

namespace Ui
{
    class AdjustParamPage;
}
class QJsonModel;
class JsonDataManager;
class HostComputeClient;

class AdjustParamPage : public QWidget
{
    Q_OBJECT

public:
    explicit AdjustParamPage(QSharedPointer<HostComputeClient> clint, QWidget *parent = nullptr);
    ~AdjustParamPage();
    /// @brief 更新json数据
    void updateJsonData();

private:
    /// @brief 消息提示
    void setMessageBox(const QString &msg, int type = 0);
    /// @brief  显示操作信息
    void setMessageOperation(const QString &msg);
    /// @brief 保存json数据
    void saveJsonData();

private:
    Ui::AdjustParamPage *ui;
    QSharedPointer<HostComputeClient> m_clint;
    JsonDataManager *m_dataManager;
    QJsonModel *m_model;
};

#endif // ADJUSTPARAMPAGE_H

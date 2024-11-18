#include "adjustparampage.h"
#include "ui_adjustparampage.h"

#include "define.h"
#include "QJsonModel.h"
#include "tarTreeDelegate.h"
#include "jsonDataManager.h"
#include "jsonDefines.h"
#include "grpcClinet.h"

#include <QDateTime>

#define MYJSON
AdjustParamPage::AdjustParamPage(QSharedPointer<HostComputeClient> clint, QWidget *parent) : QWidget(parent),
                                                                                             ui(new Ui::AdjustParamPage), m_clint(clint)
{
    ui->setupUi(this);

    m_dataManager = new JsonDataManager();
    m_model = new QJsonModel;
    m_model->setHearders(QStringList() << "调参列表"
                                       << "值");
    TarTreeDelegate *delegate = new TarTreeDelegate(this);
    ui->treeView->setModel(m_model);
    ui->treeView->setItemDelegate(delegate);
    ui->treeView->header()->resizeSection(0, 300);
    ui->treeView->setAlternatingRowColors(true);
    setStyleSheet(Define::PAGE_TREE_QSS);

    connect(ui->pushButton_update, &QPushButton::clicked, this, &AdjustParamPage::updateJsonData);
    connect(ui->pushButton_save, &QPushButton::clicked, this, &AdjustParamPage::saveJsonData);
    connect(m_model, &QJsonModel::dataValueChanged, [=](const QString &key, const QVariant &value, const QVariant &oldValue)
            {
                if(value != oldValue){
            QString str=  QString("属性[%1]的值[%2] 变更为 值[%3]").arg(key, oldValue.toString(), value.toString());
            setMessageOperation(str);} });
}

AdjustParamPage::~AdjustParamPage()
{
    delete ui;
}

void AdjustParamPage::updateJsonData()
{
    setMessageOperation("刷新轨迹列表");

#ifdef MYJSON
    int32_t task = JsonDefines::UPLOAD_PARAMS;
    std::pair<bool, std::string> reply = m_clint->ScheduleTask(task);
    QString replyQString = QString::fromStdString(reply.second);
    if (!reply.first)
    {
        setMessageBox(replyQString == "" ? "远程连接失败！" : replyQString);
        return;
    }
    QJsonDocument jsonDoc = QJsonDocument::fromJson(replyQString.toUtf8());
    if (jsonDoc.isNull())
    {
        setMessageBox("JSON 解析失败！");
        return;
    }
    QByteArray jsonData = jsonDoc.toJson();
#else
    QFile file("/home/ray/myGitproject/my-case/linux/tar/carbon_block_clean/config/adjust_params.json");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return;
    }
    QByteArray jsonData = file.readAll();
#endif
    m_dataManager->loadList(jsonData);
    QJsonDocument doc(m_dataManager->loadCurrentJsonData());
    m_model->loadJson(doc.toJson());
    setMessageBox("获取炭块列表成功！", 1);
}

void AdjustParamPage::setMessageBox(const QString &msg, int type)
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui->label_messgae->setText(QString("消息提示：[%1] %2").arg(dateTime.toString("yyyy-MM-dd hh:mm:ss")).arg(msg));
    if (type == 0)
    {
        ui->label_messgae->setStyleSheet("color: red;background:transparent;");
    }
    else if (type == 1)
    {
        ui->label_messgae->setStyleSheet("color: green;background:transparent;");
    }
}

void AdjustParamPage::setMessageOperation(const QString &msg)
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui->label_operation->setText(QString("操作提示：[%1] %2").arg(dateTime.toString("yyyy-MM-dd hh:mm:ss")).arg(msg));
    ui->label_operation->setStyleSheet("color: #ffffff;background:transparent;");
}

void AdjustParamPage::saveJsonData()
{
    setMessageOperation("保存参数列表");
    QByteArray jsonData = m_model->json();
    int32_t task = JsonDefines::UPDATA_PARAMS;
    std::pair<bool, std::string> reply = m_clint->ScheduleTask(task, jsonData.toStdString());
    QString replyQString = QString::fromStdString(reply.second);
    if (!reply.first)
    {
        setMessageBox(replyQString == "" ? "远程连接失败！" : replyQString);
        return;
    }
    setMessageBox("保存参数列表成功！", 1);
}

#include "tarpage.h"
#include "ui_tarpage.h"

#include <QFile>
#include <QDebug>
#include <QDateTime>
#include <QThread>

#include "define.h"
#include "QJsonModel.h"
#include "tarTreeDelegate.h"
#include "jsonDataManager.h"
#include "jsonDefines.h"
#include "grpcClinet.h"

#define MYJSON
TarPage::TarPage(QSharedPointer<HostComputeClient> clint, QWidget *parent) : QWidget(parent),
                                                                             ui(new Ui::TarPage), m_clint(clint)
{
    ui->setupUi(this);
    initUI();
    initConnect();
}

void TarPage::initUI()
{

    dataManager = new JsonDataManager();
    // 轨迹界面
    ui->widget_tar->hide();
    tar_model = new QJsonModel;
    TarTreeDelegate *delegate2 = new TarTreeDelegate(this);
    ui->treeView_tar->setModel(tar_model);
    ui->treeView_tar->setItemDelegate(delegate2);
    ui->treeView_tar->header()->resizeSection(0, 300);
    ui->treeView_tar->setAlternatingRowColors(true);
    setStyleSheet(Define::PAGE_TREE_QSS);

    ui->label_messgae->setText("提示信息");
    ui->pushButton_save->setVisible(false);
}

void TarPage::initConnect()
{
    connect(ui->pushButton_save, &QPushButton::clicked, this, &TarPage::saveJsonFileData);
    connect(ui->treeView, &CarTreeView::addTar, [=]()
            { ui->widget_tar->show();
                ui->comboBox->setCurrentText(dataManager->getCarbonList().first());
                switchTarItem(dataManager->getCarbonList().first()); });
    connect(ui->treeView, &CarTreeView::message, [=](QString str)
            { setMessageOperation(str); });
    connect(ui->pushButton_cancel, &QPushButton::clicked, [=]()
            { ui->widget_tar->hide();
            setMessageOperation("关闭"); });

    connect(ui->comboBox, &QComboBox::currentTextChanged, this, &TarPage::switchTarItem);

    connect(ui->pushButton_add, &QPushButton::clicked, this, &TarPage::addTarItem);

    connect(ui->pushButton_template, &QPushButton::clicked, this, &TarPage::carbonTemplate);

    connect(ui->pushButton_update, &QPushButton::clicked, this, &TarPage::carbonList);

    connect(tar_model, &QJsonModel::dataValueChanged, [=](const QString &key, const QVariant &value, const QVariant &oldValue)
            {
                if(value != oldValue){
            QString str=  QString("属性[%1]的值[%2] 变更为 值[%3]").arg(key, oldValue.toString(), value.toString());
            setMessageOperation(str);} });
}

TarPage::~TarPage()
{
    delete ui;
}

QByteArray TarPage::getJsonData()
{
    return ui->treeView->getJsonModel()->json();
}

void TarPage::setCarbonWidgetShow()
{
    if (ui->widget_tar->isVisible())
    {
        ui->widget_tar->hide();
    }
    else
    {
        ui->widget_tar->show();
    }
}

void TarPage::switchTarItem(const QString &tarName)
{
    auto jsondata = dataManager->loadCarbonProperties();
    QJsonValue bowlValue = jsondata.value(tarName);
    if (bowlValue.isObject())
    {
        QJsonObject bowlObj = bowlValue.toObject();
        QJsonDocument jsonDoc(bowlObj);
        QByteArray jsonData = jsonDoc.toJson();
        tar_model->loadJson(jsonData);
    }
}

void TarPage::addTarItem()
{
    QString tarname = ui->comboBox->currentText();
    setMessageOperation(QString("添加轨迹(%1)").arg(tarname));
    QByteArray jsonData = tar_model->json();
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
    QJsonObject jsonObj = jsonDoc.object();
    ui->treeView->addTarItem(jsonObj, tarname);
}

void TarPage::carbonTemplate()
{
    setMessageOperation("获取炭块模版");
    ui->comboBox->clear();
    ui->pushButton_template->setEnabled(false);

#ifdef MYJSON
    int32_t task = JsonDefines::UPLOAD_TEMPLATE;
    auto reply = m_clint->ScheduleTask(task);
    QString replyQString = QString::fromStdString(reply.second);
    getTranslation();
    if (!reply.first)
    {
        ui->pushButton_template->setEnabled(true);
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
    QFile file("/home/ray/myGitproject/my-case/linux/tar/carbon_block_clean/config/i.json");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return;
    }
    QByteArray jsonData = file.readAll();
#endif

    dataManager->load(jsonData);
    QJsonDocument doc(dataManager->loadCurrentJsonData());
    ui->treeView->getJsonModel()->loadJson(doc.toJson());
    ui->treeView->setTarList(dataManager->getCarbonList());
    ui->comboBox->addItems(dataManager->getCarbonList());
    ui->pushButton_template->setEnabled(true);
    ui->pushButton_save->setVisible(true);
    ui->treeView->setEditable(true);
    ui->widget_tar->show();
    ui->treeView->getJsonModel()->setHearders(QStringList() << "炭块模版"
                                                            << "值");
    setMessageBox("获取炭块模版成功！", 1);
}

void TarPage::carbonList()
{
    setMessageOperation("刷新轨迹列表");
    ui->pushButton_update->setEnabled(false);

#ifdef MYJSON
    int32_t task = JsonDefines::UPLOAD_TRAJ;
    auto reply = m_clint->ScheduleTask(task);
    QString replyQString = QString::fromStdString(reply.second);
    if (!reply.first)
    {
        ui->pushButton_update->setEnabled(true);
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
    QFile file("/home/ray/myGitproject/my-case/linux/tar/carbon_block_clean/config/j.json");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return;
    }
    QByteArray jsonData = file.readAll();
#endif
    dataManager->loadList(jsonData);
    QJsonDocument doc(dataManager->loadCurrentJsonData());
    ui->treeView->getJsonModel()->loadJson(doc.toJson());
    ui->pushButton_update->setEnabled(true);
    ui->treeView->setEditable(false);
    ui->pushButton_save->setVisible(false);
    ui->widget_tar->hide();
    ui->treeView->getJsonModel()->setHearders(QStringList() << "炭块列表"
                                                            << "值");
    setMessageBox("获取炭块列表成功！", 1);
}
void TarPage::saveJsonFileData()
{
    setMessageOperation("生成炭块");
    QByteArray jsonData = ui->treeView->getJsonModel()->json();
    // if (!nameDetection(jsonData))
    //     return;
    int32_t task = JsonDefines::CREATE_TRAJ;
    auto reply = m_clint->ScheduleTask(task, jsonData.toStdString());
    QString replyQString = QString::fromStdString(reply.second);
    if (!reply.first)
    {
        ui->pushButton_save->setEnabled(true);
        setMessageBox(replyQString == "" ? "远程连接失败！" : replyQString);
        return;
    }
    setMessageBox("炭块生成成功！", 1);
}

void TarPage::setMessageBox(const QString &msg, int type)
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

void TarPage::setMessageOperation(const QString &msg)
{
    QDateTime dateTime = QDateTime::currentDateTime();
    ui->label_operation->setText(QString("操作提示：[%1] %2").arg(dateTime.toString("yyyy-MM-dd hh:mm:ss")).arg(msg));
    ui->label_operation->setStyleSheet("color: #ffffff;background:transparent;");
}

bool TarPage::nameDetection(QByteArray &data)
{
    auto listName = dataManager->getTarList();
    QJsonDocument jsonDoc = QJsonDocument::fromJson(data);
    QJsonObject obj = jsonDoc.object();
    if (obj.contains(JsonDefines::CARBONINFO) && obj[JsonDefines::CARBONINFO].isObject())
    {
        QJsonObject carbonInfo = obj[JsonDefines::CARBONINFO].toObject();
        if (carbonInfo.contains(JsonDefines::NAME) && carbonInfo[JsonDefines::NAME].isString())
        {
            QString name = carbonInfo[JsonDefines::NAME].toString();
            if (listName.contains(name))
            {
                setMessageBox("炭块名称重复！，请修改后重试！");
                return false;
            }
        }
    }
    return true;
}

void TarPage::getTranslation()
{
    int32_t translation = JsonDefines::CN_LIST;
    auto translation_reply = m_clint->ScheduleTask(translation);
    QString replyQString = QString::fromStdString(translation_reply.second);
    QJsonDocument jsonDoc = QJsonDocument::fromJson(replyQString.toUtf8());
    if (jsonDoc.isNull())
    {
        setMessageBox("JSON 解析失败！");
        return;
    }
    QByteArray jsonData = jsonDoc.toJson();
    // QFile file("/home/ray/myGitproject/my-case/linux/tar/carbon_block_clean/config/save.json");
    // if (!file.open(QIODevice::WriteOnly))
    // {
    //     qDebug() << "Error: Could not open file for writing.";
    //     return;
    // }

    // file.write(jsonData);
    // file.close();
}

#include "myXml.h"

#include <QDomDocument>
#include <QDomElement>
#include <QDomNodeList>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>

QString filepath = "D:\\GitProject\\my-case\\QT\\read_write_xml\\my.xml";
BoardSettingWidget::BoardSettingWidget(QWidget *p) : QDialog(p)
{
    initUI();
    readCurrentXmlContent();
}

BoardSettingWidget::~BoardSettingWidget() {}

void BoardSettingWidget::initUI()
{
    QGroupBox *groupBox = new QGroupBox(this);
    groupBox->setTitle("DJ");

    QVBoxLayout *layout = new QVBoxLayout(groupBox);

    checkBoxEnable = new QCheckBox("enable", groupBox);
    checkBoxEnable->setChecked(true);
    layout->addWidget(checkBoxEnable);

    checkBoxCustom = new QCheckBox("custom", groupBox);
    layout->addWidget(checkBoxCustom);

    QHBoxLayout *hlayout1 = new QHBoxLayout(groupBox);
    QSpacerItem *spacerItemLine = new QSpacerItem(80, 20, QSizePolicy::Expanding, QSizePolicy::Maximum);
    lineEdit = new QLineEdit(groupBox);
    lineEdit->setPlaceholderText("intput");
    lineEdit->setVisible(false);
    hlayout1->addWidget(lineEdit);
    hlayout1->addSpacerItem(spacerItemLine);

    QHBoxLayout *hlayout2 = new QHBoxLayout(groupBox);
    QSpacerItem *spacerItemBox = new QSpacerItem(80, 20, QSizePolicy::Expanding, QSizePolicy::Maximum);
    comboBox = new QComboBox(groupBox);
    comboBox->addItem("5");
    comboBox->addItem("10");
    comboBox->addItem("20");
    comboBox->addItem("30");
    comboBox->setVisible(true); // 默认显示
    hlayout2->addWidget(comboBox);
    hlayout2->addSpacerItem(spacerItemBox);

    layout->addLayout(hlayout2);
    layout->addLayout(hlayout1);

    groupBox->setLayout(layout);

    QSpacerItem *spacerItem = new QSpacerItem(80, 20, QSizePolicy::Maximum, QSizePolicy::Expanding);
    QVBoxLayout *layoutr = new QVBoxLayout(this);
    layoutr->addWidget(groupBox);
    layoutr->addItem(spacerItem);
    setLayout(layoutr);

    connect(checkBoxCustom, &QCheckBox::stateChanged, [=](int state)
            {
        if (state == Qt::Checked) {
            lineEdit->setVisible(true);
            comboBox->setVisible(false);
        } else {

            lineEdit->setVisible(false);
            comboBox->setVisible(true);
        } });
    connect(checkBoxEnable, &QCheckBox::stateChanged, [=](int state)
            {
        if (state == Qt::Checked) {
            checkBoxCustom->setEnabled(true);
            lineEdit->setEnabled(true);
            comboBox->setEnabled(true);
        } else {
            checkBoxCustom->setEnabled(false);
            lineEdit->setEnabled(false);
            comboBox->setEnabled(false);
        } });
}

void BoardSettingWidget::readCurrentXmlContent()
{
    // 读文件
    QFile file(filepath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file";
        return;
    }
    QDomDocument doc;
    if (!doc.setContent(&file))
    {
        qDebug() << "Failed to parse the file into a DOM tree.";
        return;
    }

    QDomElement root = doc.documentElement();
    QDomNodeList nodes = root.childNodes();
    for (int i = 0; i < nodes.length(); i++)
    {
        QDomElement node = nodes.at(i).toElement();
        if (node.tagName() == "enable")
        {
            checkBoxEnable->setChecked(node.text().toInt() == 2);
        }
        else if (node.tagName() == "custom")
        {
            checkBoxCustom->setChecked(node.text().toInt() == 2);
        }
        else if (node.tagName() == "line")
        {
            lineEdit->setText(node.text());
        }
        else if (node.tagName() == "combox")
        {
            comboBox->setCurrentIndex(node.text().toInt());
        }
    }
}

void BoardSettingWidget::wirteCurrentXmlContent()
{
    QString content;
    QDomDocument doc;
    // 设置XML声明和编码
    doc.appendChild(doc.createProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\""));
    QDomElement root = doc.createElement("BoardSettingWidget");

    // 从QComboBox获取当前选中的索引或者文本
    QDomElement comboxElement = doc.createElement("combox");
    QString comboxText = QString::number(comboBox->currentIndex());
    QDomText comboxValue = doc.createTextNode(comboxText);
    comboxElement.appendChild(comboxValue);

    // 从QLineEdit获取文本
    QDomElement lineElement = doc.createElement("line");
    QString lineText = lineEdit->text();
    QDomText lineValue = doc.createTextNode(lineText);
    lineElement.appendChild(lineValue);

    // 从check获取文本
    QDomElement check = doc.createElement("enable");
    QString checkstr = QString::number(checkBoxEnable->checkState());
    QDomText checkvalue = doc.createTextNode(checkstr);
    check.appendChild(checkvalue);

    QDomElement check1 = doc.createElement("custom");
    QString checkstr1 = QString::number(checkBoxCustom->checkState());
    QDomText checkvalue1 = doc.createTextNode(checkstr1);
    check1.appendChild(checkvalue1);

    // 将元素添加到根节点
    root.appendChild(comboxElement);
    root.appendChild(lineElement);
    root.appendChild(check);
    root.appendChild(check1);

    // 将根元素添加到文档
    doc.appendChild(root);

    // 打开文件
    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file for writing";
        return;
    }

    // 写入文件
    QTextStream stream(&file);
    stream << doc.toString();
    file.close();
}

// <?xml version=" 1.0 " encoding="UTF-8"?>
// <BoardSettingWidget>
//     <combox> 2 </combox>
//     <line> this a test</line>
//     <enable> 1</enable>
//     <custom> 0</custom>
// </BoardSettingWidget>
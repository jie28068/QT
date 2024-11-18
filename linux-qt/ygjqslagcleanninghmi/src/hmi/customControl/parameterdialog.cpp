#include "parameterdialog.h"
#include "ui_parameterdialog.h"
#include "globals.h"
#include "qtnode.h"
#include <QRegExpValidator>
#include <QRegExp>

ParameterDialog::ParameterDialog(QtNode *ptr, QWidget *parent) : QDialog(parent), node_ptr(ptr),
                                                                 ui(new Ui::ParameterDialog)
{
        ui->setupUi(this);
        setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);

        QRegExp regExp("^([1-9]?[0-9](\\.[0-9])?|1[0-9]{2}(\\.[0-9])?|2[0-9]{2}(\\.[0-9])?|200(\\.0{1,2})?)$");

        QRegExpValidator *validator = new QRegExpValidator(regExp);
        ui->lineEdit->setValidator(validator);
        ui->lineEdit_2->setValidator(validator);
        ui->lineEdit_3->setValidator(validator);
        ui->lineEdit_4->setValidator(validator);

        connect(ui->pushButton, &QPushButton::clicked, this, &QDialog::accept);
        connect(ui->pushButton_4, &QPushButton::clicked, this, [=]()
                { node_ptr->set_system_params(globals::workTimeUpdate, QStringList() << ui->lineEdit->text());    
                       if(line){ ui->lineEdit->setStyleSheet("color: yellow; font-size: 16pt;"); line = false;} });
        connect(ui->pushButton_3, &QPushButton::clicked, this, [=]()
                {       
                        if(lineX){
                        ui->lineEdit_2->setStyleSheet("color: yellow; font-size: 16pt;"); lineX =false;}
                        if(lineY){
                        ui->lineEdit_3->setStyleSheet("color: yellow; font-size: 16pt;"); lineY =false;}
                        if(lineZ){
                        ui->lineEdit_4->setStyleSheet("color: yellow; font-size: 16pt;"); lineZ =false;}
                        node_ptr->set_system_params(globals::toolOffsetUpdate, QStringList() << ui->lineEdit_2->text() << ui->lineEdit_3->text() << ui->lineEdit_4->text()); });
        connect(ui->pushButton_2, &QPushButton::clicked, this, [=]()
                { node_ptr->set_system_params(globals::systemParamsLoad, QStringList());
                        this->setStyleSheet("QDialog { border-image: url(:/resource/svg/作业次数-背景框.svg); } "
                        "QLabel { color: white; font-size: 16pt; margin: 10px; background-color: transparent; } "
                        "QLineEdit { color: white; font-size: 16pt; } ");
                        ui->lineEdit->setStyleSheet("color: white; font-size: 16pt;"); 
                        ui->lineEdit_2->setStyleSheet("color: white; font-size: 16pt;"); 
                        ui->lineEdit_3->setStyleSheet("color: white; font-size: 16pt;"); 
                        ui->lineEdit_4->setStyleSheet("color: white; font-size: 16pt;"); });

        connect(node_ptr, &QtNode::timer_signal, [=](const QString &str)
                { ui->lineEdit->setText(str); });
        connect(node_ptr, &QtNode::coordinate_signal, [=](const QStringList &str)
                { 
                if(str.size() == 3){
                        ui->lineEdit_2->setText(str.at(0)); 
                        ui->lineEdit_3->setText(str.at(1)); 
                        ui->lineEdit_4->setText(str.at(2));
                } });
        /// 先后影响连接
        ui->pushButton_2->click();
        connect(ui->lineEdit, &QLineEdit::textChanged, [=](const QString &str)
                { ui->lineEdit->setStyleSheet("color: red; font-size: 16pt;");
                line = true; });
        connect(ui->lineEdit_2, &QLineEdit::textChanged, [=](const QString &str)
                { ui->lineEdit_2->setStyleSheet("color: red; font-size: 16pt;");
                lineX = true; });
        connect(ui->lineEdit_3, &QLineEdit::textChanged, [=](const QString &str)
                { ui->lineEdit_3->setStyleSheet("color: red; font-size: 16pt;"); 
                lineY = true; });
        connect(ui->lineEdit_4, &QLineEdit::textChanged, [=](const QString &str)
                { ui->lineEdit_4->setStyleSheet("color: red; font-size: 16pt;"); 
                lineZ = true; });
}

ParameterDialog::~ParameterDialog()
{
        delete ui;
}

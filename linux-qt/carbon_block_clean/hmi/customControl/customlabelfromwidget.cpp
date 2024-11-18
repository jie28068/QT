#include "customlabelfromwidget.h"
#include "ui_customlabelfromwidget.h"

#include "define.h"
#include "cuelightlabel.h"

CustomLabelFromWidget::CustomLabelFromWidget(QWidget *parent) : QWidget(parent),
                                                                ui(new Ui::CustomLabelFromWidget)
{
    ui->setupUi(this);
}

CustomLabelFromWidget::~CustomLabelFromWidget()
{
    delete ui;
}

void CustomLabelFromWidget::setLabelIcon(const QString &icon)
{
    ui->label_icon->setStyleSheet(QString("%1;background-repeat: no-repeat;background-position: center;").arg(icon));
}

void CustomLabelFromWidget::setLabelTitle(const QString &title)
{
    ui->label_title->setText(title);
}

void CustomLabelFromWidget::setLabelText(const QString &text)
{
    if (!ui->label_text)
    {
        return;
    }
    ui->label_text->setText(text);
    if (text == Define::STATUS_ONLINE || text == Define::STATUS_ONPLACE || text == Define::STATUS_RUN || text == Define::STATUS_SAFEY || text == Define::STATUS_OFFDOOR)
    {
        ui->label_pixmap->setStatus(CueLightLabel::Online);
        ui->label_pixmap->setVisible(true);
        ui->label_text->setStyleSheet("color: rgb(0, 255, 0);");
    }
    else if (text == Define::STATUS_OFFLINE || text == Define::STATUS_OFFPLACE || text == Define::STATUS_ERROR || text == Define::STATUS_STOP || text == Define::STATUS_NOSAFEY || text == Define::STATUS_OPENDOOR)
    {
        ui->label_pixmap->setStatus(CueLightLabel::Offline);
        ui->label_pixmap->setVisible(true);
        ui->label_text->setStyleSheet("color: rgb(255, 0, 0);");
    }
    else if (text == Define::STATUS_UNKNOWN)
    {
        ui->label_pixmap->setVisible(false);
        ui->label_text->setStyleSheet("color:rgb(128, 128, 128);");
    }
    else
    {
        ui->label_pixmap->setVisible(false);
        ui->label_text->setStyleSheet("color: rgb(0, 255, 255);");
    }
}

void CustomLabelFromWidget::setLabelPixmap(const QString &pixmap)
{
    ui->label_pixmap->setStyleSheet(pixmap);
}

void CustomLabelFromWidget::setbreathingLight(bool falg)
{
    ui->label_pixmap->setBreathingEffectEnabled(falg);
}

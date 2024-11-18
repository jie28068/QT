#include "customButton.h"
#include "define.h"

#include <QVBoxLayout>
#include <QMouseEvent>
#include <QStyle>

CustomButton::CustomButton(QWidget *parent) : QPushButton(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setSpacing(0);
    layout->setMargin(0);

    imageLabel = new QLabel(this);
    imageLabel->setAlignment(Qt::AlignCenter);
    descriptionLabel = new QLabel(this);
    descriptionLabel->setAlignment(Qt::AlignCenter);

    layout->addWidget(imageLabel, 1);
    layout->addWidget(descriptionLabel, 0);
    // setFlat(true); // 设置按钮扁平化，没有边框

    setStyleSheet("background:transparent;border:none;");
}

void CustomButton::setImage(const QPixmap &image)
{
    imageLabel->setPixmap(image);
}

void CustomButton::setDescription(const QString &text)
{
    descriptionLabel->setText(text);
}

void CustomButton::setPageWidget(QWidget *page)
{
    targetPage = page;
}

void CustomButton::setHighlighted(bool highlighted)
{
    if (highlighted)
    {
        imageLabel->setPixmap(m_highlightedPixmap);
        descriptionLabel->setStyleSheet("QLabel{background:transparent; color: rgb(255,255,255);font-size:16px;font-weight: bold;}");
    }
    else
    {
        imageLabel->setPixmap(m_pixmap);
        descriptionLabel->setStyleSheet("QLabel{background:transparent; color: rgb(128,138,135);font-size:16px;font-weight: bold;}");
    }
}

void CustomButton::setPushButtonStyle(bool flag)
{
    if (flag)
    {
        imageLabel->setPixmap(QPixmap(":/images/resources/qidong.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        descriptionLabel->setStyleSheet("QLabel{background:transparent; color: rgb(255,255,255);font-size:16px;font-weight: bold;}");
    }
    else
    {
        imageLabel->setPixmap(QPixmap(":/images/resources/stop.png").scaled(QSize(Define::PAGE_ICON_SIZE, Define::PAGE_ICON_SIZE), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        descriptionLabel->setStyleSheet("QLabel{background:transparent; color: rgb(128,138,135);font-size:16px;font-weight: bold;}");
    }
}

void CustomButton::setTextCo(const QString &text)
{
    descriptionLabel->setText(text);
}

void CustomButton::setTextIcon(const QString &text, const QPixmap &icon, const QPixmap &highlighted)
{
    descriptionLabel->setText(text);
    imageLabel->setPixmap(icon);

    m_pixmap = icon;
    m_highlightedPixmap = highlighted;

    descriptionLabel->setStyleSheet("QLabel{background:transparent; color: rgb(128,138,135);font-size:16px;font-weight: bold;}");
}

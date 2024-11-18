#include "customlabelfrom.h"
#include "cuelightlabel.h"
#include "globals.h"

#include <QHBoxLayout>
CustomLabelFrom::CustomLabelFrom(QWidget *parent) : QWidget(parent)
{
    setStyleSheet("border: none; background: transparent;");
    QHBoxLayout *layout = new QHBoxLayout(this);

    label_icon = new QLabel(this);
    label_title = new QLabel(this);
    label_text = new QLabel(this);
    label_pixmap = new CueLightLabel(this);

    layout->addWidget(label_icon);
    layout->addWidget(label_title);
    layout->addWidget(label_text);
    layout->addWidget(label_pixmap);

    label_title->setAlignment(Qt::AlignCenter);
    label_text->setAlignment(Qt::AlignCenter);

    // label_icon->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    // label_icon->setMinimumSize(30, 30);
    // label_title->setMinimumSize(30, 30);
    // label_title->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    // // label_title->setMinimumHeight(30);
    // label_text->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    // label_text->setMinimumSize(30, 30);
    // // label_text->setMinimumHeight(30);
    // // label_pixmap->setMinimumHeight(30);
    // label_pixmap->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    // label_pixmap->setMinimumSize(30, 30);

    label_icon->setFixedWidth(30);
    label_title->setFixedWidth(120);
    label_title->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);
    label_title->setMinimumHeight(30);
    label_text->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    label_text->setMinimumHeight(30);
    label_pixmap->setFixedWidth(30);

    label_title->setStyleSheet("color: rgb(255, 255, 255);");

    this->layout()->setContentsMargins(5, 5, 5, 5);
    setStyleSheet("QLabel{font-size: 20px;}");
}

void CustomLabelFrom::adjustFont(QLabel *lb)
{
    if (!lb)
        return;

    QFont font = lb->font();
    QFontMetrics fontMetrics(font);
    int minPointSize = 1; // 设置最小的字号限制
    bool needAdjustment = true;

    // 获取QLabel的尺寸，考虑边框和边距
    QRect contentRect = lb->contentsRect();

    // 计算当前字体下文本的宽度和高度
    int nFontWidth = fontMetrics.horizontalAdvance(lb->text());
    int nFontHeight = fontMetrics.boundingRect(lb->text()).height();

    while (needAdjustment && font.pointSize() > minPointSize)
    {
        if (nFontWidth > contentRect.width() || nFontHeight > contentRect.height())
        {
            // 字体大小减小1
            font.setPointSize(font.pointSize() - 1);
            fontMetrics = QFontMetrics(font);
            nFontWidth = fontMetrics.horizontalAdvance(lb->text());
            nFontHeight = fontMetrics.boundingRect(lb->text()).height();
        }
        else
        {
            needAdjustment = false;
        }
    }

    lb->setFont(font);
}

void CustomLabelFrom::setLabelIcon(const QString &icon)
{
    label_icon->setStyleSheet(QString("%1;background-repeat: no-repeat;background-position: center;").arg(icon));
}

void CustomLabelFrom::setLabelTitle(const QString &title)
{
    label_title->setText(title);
    // adjustFont(label_title);
}

void CustomLabelFrom::setLabelText(const QString &text)
{
    label_text->setText(text);
    if (text == globals::online || text == globals::on || text == globals::enable || text == globals::running)
    {
        label_pixmap->setStatus(CueLightLabel::Online);
        label_text->setStyleSheet("color: rgb(0, 255, 0);");
    }
    else if (text == globals::offline || text == globals::abnormal || text == globals::emergencyStop || text == globals::disable || text == globals::stop)
    {
        label_pixmap->setStatus(CueLightLabel::Offline);
        label_text->setStyleSheet("color: rgb(255, 0, 0);");
    }
    else if (text == globals::dropped || text == globals::closed)
    {
        label_pixmap->setStatus(CueLightLabel::Dropped);
        label_text->setStyleSheet("color: rgb(128, 128, 128);");
    }
    else if (text == globals::droppedDo)
    {
        label_pixmap->setVisible(false);
        label_text->setStyleSheet("color:rgb(128, 128, 128);");
    }
    else
    {
        label_pixmap->setVisible(false);
        label_text->setStyleSheet("color: rgb(0, 255, 255);");
    }
    // adjustFont(label_text);
}

void CustomLabelFrom::setLabelPixmap(const QString &pixmap)
{
    label_pixmap->setStyleSheet(pixmap);
}

void CustomLabelFrom::setbreathingLight(bool falg)
{
    label_pixmap->setBreathingEffectEnabled(falg);
}

QLabel *CustomLabelFrom::getLabelIcon()
{
    return label_icon;
}

QLabel *CustomLabelFrom::getLabelTitle()
{
    return label_title;
}

QLabel *CustomLabelFrom::getLabelText()
{
    return label_text;
}

CueLightLabel *CustomLabelFrom::getLabelPixmap()
{
    return label_pixmap;
}

void CustomLabelFrom::securityDoorText(const QString &text)
{
    label_text->setText(text);
    if (text == globals::notclosed)
    {
        label_pixmap->setStatus(CueLightLabel::Offline);
        label_text->setStyleSheet("color: rgb(255, 0, 0);");
    }
    else if (text == globals::unlocked)
    {
        label_pixmap->setStatus(CueLightLabel::other);
        label_text->setStyleSheet("color: rgb(255, 255, 0);");
    }
    else if (text == globals::islocked)
    {
        label_pixmap->setStatus(CueLightLabel::Online);
        label_text->setStyleSheet("color: rgb(0, 255, 0);");
    }
    else if (text == globals::abnormal || text == globals::dropped)
    {
        label_pixmap->setStatus(CueLightLabel::Dropped);
        label_text->setStyleSheet("color: rgb(128, 128, 128);");
    }
}

void CustomLabelFrom::setWidgetToolTip(const QString &tip)
{
    label_icon->setToolTip(tip);
    label_title->setToolTip(tip);
    label_text->setToolTip(tip);
    // label_pixmap->setToolTip(tip);
}

CueLightLabelFrom::CueLightLabelFrom(QWidget *parent) : QWidget(parent)
{
    setStyleSheet("border: none; background: transparent;");
    QHBoxLayout *layout = new QHBoxLayout(this);
    label_icon = new QLabel(this);
    label_title = new QLabel(this);
    label_status1 = new CueLightLabel(this);
    label_status2 = new CueLightLabel(this);
    label_status3 = new CueLightLabel(this);

    label_title->setAlignment(Qt::AlignCenter);

    layout->addWidget(label_icon);
    layout->addWidget(label_title);
    layout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Minimum));
    layout->addWidget(label_status1);
    layout->addSpacerItem(new QSpacerItem(20, 10, QSizePolicy::Fixed, QSizePolicy::Minimum));
    layout->addWidget(label_status2);
    layout->addSpacerItem(new QSpacerItem(20, 10, QSizePolicy::Fixed, QSizePolicy::Minimum));
    layout->addWidget(label_status3);
    // layout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Minimum));

    label_icon->setFixedWidth(30);
    label_title->setFixedWidth(120);
    label_title->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);
    label_title->setMinimumHeight(30);
    label_title->setStyleSheet("color: rgb(255, 255, 255);");
    label_status1->setFixedWidth(30);
    label_status2->setFixedWidth(30);
    label_status3->setFixedWidth(30);

    this->layout()->setContentsMargins(5, 5, 5, 5);
    setStyleSheet("QLabel{font-size: 20px;}");
}

void CueLightLabelFrom::setLabelIcon(const QString &icon)
{
    label_icon->setStyleSheet(QString("%1;background-repeat: no-repeat;background-position: center;").arg(icon));
}

void CueLightLabelFrom::setLabelTitle(const QString &title)
{
    label_title->setText(title);
}

void CueLightLabelFrom::setToolTipStr(const QString &str1, const QString &str2, const QString &str3)
{
    label_status1->setToolTip(str1);
    label_status2->setToolTip(str2);
    label_status3->setToolTip(str3);
}

void CueLightLabelFrom::setLightStatus(int index, const QString &status)
{
    if (status == globals::dropped)
    {
        label_status1->setStatus(CueLightLabel::Dropped);
        label_status2->setStatus(CueLightLabel::Dropped);
        label_status3->setStatus(CueLightLabel::Dropped);
    }
    else if (status == globals::online)
    {
        if (index == 1)
        {
            label_status1->setStatus(CueLightLabel::Online);
        }
        else if (index == 2)
        {
            label_status2->setStatus(CueLightLabel::Online);
        }
        else if (index == 3)
        {
            label_status3->setStatus(CueLightLabel::Online);
        }
    }
    else if (status == globals::offline)
    {
        if (index == 1)
        {
            label_status1->setStatus(CueLightLabel::Offline);
        }
        else if (index == 2)
        {
            label_status2->setStatus(CueLightLabel::Offline);
        }
        else if (index == 3)
        {
            label_status3->setStatus(CueLightLabel::Offline);
        }
    }
}

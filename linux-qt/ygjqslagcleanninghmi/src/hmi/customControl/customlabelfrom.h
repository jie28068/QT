#ifndef CUSTOMLABELFROM_H
#define CUSTOMLABELFROM_H

#include <QObject>
#include <QWidget>
#include <QString>
#include <QLabel>

class CueLightLabel;

class CustomLabelFrom : public QWidget
{
    Q_OBJECT
public:
    CustomLabelFrom(QWidget *parent = nullptr);

    void setLabelIcon(const QString &icon);
    void setLabelTitle(const QString &title);
    void setLabelText(const QString &text);
    void setLabelPixmap(const QString &pixmap);
    void setbreathingLight(bool falg);

    QLabel *getLabelIcon();
    QLabel *getLabelTitle();
    QLabel *getLabelText();
    CueLightLabel *getLabelPixmap();

    /// 安全门的状态特殊处理
    void securityDoorText(const QString &text);

    void setWidgetToolTip(const QString &tip);

private:
    void adjustFont(QLabel *lb);

private:
    QLabel *label_icon;          // 图标
    QLabel *label_title;         // 功能
    QLabel *label_text;          // 内容
    CueLightLabel *label_pixmap; // 状态
};

class CueLightLabelFrom : public QWidget
{
public:
    CueLightLabelFrom(QWidget *parent = nullptr);
    void setLabelIcon(const QString &icon);
    void setLabelTitle(const QString &title);
    void setToolTipStr(const QString &str1, const QString &str2, const QString &str3);
    void setLightStatus(int index, const QString &status);

private:
    QLabel *label_icon;  // 图标
    QLabel *label_title; // 功能
    CueLightLabel *label_status1;
    CueLightLabel *label_status2;
    CueLightLabel *label_status3;
};
#endif
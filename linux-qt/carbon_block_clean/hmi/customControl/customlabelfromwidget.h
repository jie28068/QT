#ifndef CUSTOMLABELFROMWIDGET_H
#define CUSTOMLABELFROMWIDGET_H

#include <QWidget>

namespace Ui
{
    class CustomLabelFromWidget;
}

class CustomLabelFromWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CustomLabelFromWidget(QWidget *parent = nullptr);
    void setLabelIcon(const QString &icon);
    void setLabelTitle(const QString &title);
    /// @brief 信息设置
    /// @param text
    void setLabelText(const QString &text);
    void setLabelPixmap(const QString &pixmap);
    void setbreathingLight(bool falg);
    ~CustomLabelFromWidget();

private:
    Ui::CustomLabelFromWidget *ui;
};

#endif // CUSTOMLABELFROMWIDGET_H

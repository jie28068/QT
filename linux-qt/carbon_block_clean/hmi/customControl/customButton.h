#pragma once

#include <QPushButton>
#include <QPixmap>
#include <QLabel>

class CustomButton : public QPushButton
{
    Q_OBJECT

public:
    explicit CustomButton(QWidget *parent = nullptr);

    void setImage(const QPixmap &image);
    void setDescription(const QString &text);
    void setPageWidget(QWidget *page); // 设置点击后要显示的页面
    void setHighlighted(bool highlighted);
    /// @brief 设置图片样式
    /// @param flag
    void setPushButtonStyle(bool flag);
    /// @brief 设置文本内容
    /// @param text
    void setTextCo(const QString &text);
    void setTextIcon(const QString &text, const QPixmap &icon, const QPixmap &highlighted = QPixmap());
signals:
    void buttonClicked(QWidget *page); // 发出信号，传递要显示的页面

private:
    QLabel *imageLabel;       // 显示图片的标签
    QLabel *descriptionLabel; // 显示描述文本的标签
    QWidget *targetPage;      // 点击后要跳转到的页面

    QPixmap m_pixmap;
    QPixmap m_highlightedPixmap;
};
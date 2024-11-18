#ifndef CUELIGHTLABEL_H
#define CUELIGHTLABEL_H

#include <QLabel>
#include <QPropertyAnimation>

class CueLightLabel : public QLabel
{
    Q_OBJECT
    Q_PROPERTY(QColor color READ getColor WRITE setColor)
public:
    enum CueLightStatus
    {
        Online = 0, // 在线
        Offline,    // 离线
        Dropped,    // 掉线
        other,
    };

    explicit CueLightLabel(QWidget *parent = nullptr);
    ~CueLightLabel();

    /// @brief 设置状态
    void setStatus(CueLightStatus status);

    /// @brief 设置闪烁效果
    /// @param enabled
    void setBreathingEffectEnabled(bool enabled);

    QColor getColor() const { return m_color; }
    void setColor(QColor color) { m_color = color; }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    CueLightStatus mCurrentStatus;
    QPropertyAnimation *mAnimation = nullptr;
    QColor m_color;
};

#endif
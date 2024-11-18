#ifndef CONTROLLINEEDIT_H
#define CONTROLLINEEDIT_H

#include "globalDefinition.h"
#include "controlBase.h"

#include <QObject>

class ControlOneToMany : public ControlBase
{
    Q_OBJECT

    Q_PROPERTY(QString m_controlName READ controlName WRITE setControlName)
    Q_PROPERTY(QString m_controlValue READ controlValue WRITE setControlValue);
    Q_PROPERTY(QString m_controlDisplayName READ controlDisplayName WRITE setControlDisplayName);
    Q_PROPERTY(bool m_controlVisiblePort READ controlVisiblePort WRITE setControlVisiblePort)
    Q_PROPERTY(ControlLimitingType m_controlLimitingType READ controlLimitingType WRITE setControlLimitingType)
    Q_PROPERTY(ControlVisibleType m_controlVisibleType READ controlVisibleType WRITE setControlVisibleType)
    Q_PROPERTY(ControlType m_controlType READ controlType WRITE setControlType)
    Q_PROPERTY(QString m_controlTypeValue READ controlTypeValue WRITE setControlTypeValue)
    Q_PROPERTY(QString m_controlPrompt READ controlPrompt WRITE setControlPrompt)

public:
    explicit ControlOneToMany(QObject *parent = nullptr);

    enum ControlLimitingType
    {
        NoLimiting,
        UpperLimitAmplitude,
        LowerLimitAmplitude,
    };
    Q_ENUM(ControlLimitingType)

    enum ControlVisibleType
    {
        Invisible,
        VisibleEditable,
        VisibleNoEditable,
    };
    Q_ENUM(ControlVisibleType)

    enum ControlType
    {
        ComplexNumber,          // 复数
        Font,                   // 字体
        Int,                    // 整型
        Double,                 // 浮点型
        Date,                   // 日期
        Color,                  // 颜色
        ComboBox,               // 下拉框
        TextBox,                // 文本框
        CheckBox,               // 勾选框
        Write,                  // 写文件
        Read,                   // 读文件
        IntVector,              // 整型数组
        DoubleVector,           // 浮点型数组
        MathematicalExpression, // 数学表达式
        CanEnterDropDownBox,    // 可输入下拉框
        DropDownBoxWithPreset,  // 带预设项下拉框

    };
    Q_ENUM(ControlType)

    int rtti() const;
    static int RTTI;

    QString name() override;

    virtual QString getName();
    virtual void setName(const QString &str);

    QString controlName();
    void setControlName(QString name);

    QString controlValue();
    void setControlValue(QString value);

    QString controlDisplayName();
    void setControlDisplayName(QString displayName);

    bool controlVisiblePort();
    void setControlVisiblePort(bool visiblePort);

    ControlLimitingType controlLimitingType();
    void setControlLimitingType(ControlLimitingType limitingType);

    ControlVisibleType controlVisibleType();
    void setControlVisibleType(ControlVisibleType visibleType);

    ControlType controlType();
    void setControlType(ControlType type);

    QString controlTypeValue();
    void setControlTypeValue(QString typeValue);

    QString controlPrompt();
    void setControlPrompt(QString prompt);

private:
    QString m_name;                            // 控件名称
    QString m_controlName;                     // 名称
    QString m_controlValue;                    // 变量值
    QString m_controlDisplayName;              // 显示名称
    bool m_controlVisiblePort;                 // 端口可见
    ControlLimitingType m_controlLimitingType; // 限幅类型
    ControlVisibleType m_controlVisibleType;   // 可见类型
    ControlType m_controlType;                 // 控件类型
    QString m_controlTypeValue;                // 控件值
    QString m_controlPrompt;                   // 变量说明
};
Q_DECLARE_METATYPE(ControlOneToMany *)
#endif
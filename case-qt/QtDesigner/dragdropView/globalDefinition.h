#ifndef GLOBALDEFINITION_H
#define GLOBALDEFINITION_H

#include <QString>
#include <QStringList>
#include <QVariantMap>
#include <QDebug>

class ControlBase;
namespace GlobalDefinition
{
    struct ControlModel
    {
        ControlBase *control;
        QString name;
    };

    static const int controlModelName = Qt::UserRole + 2;   // 名称
    static const int controlModelPixmap = Qt::UserRole + 3; // 图标
    static const int controlModelType = Qt::UserRole + 4;   // 类型

    static const QString controlMimeType = "control/x-qabstractitemmodeldatalist";
    static const QString controlDialogMimeType = "control/y-qabstractitemmodeldatalist";
    // 属性
    static const QString controlName = "controlName";                 // 名称(封装后的控件)
    static const QString controlValue = "controlValue";               // 变量值
    static const QString controlDisplayName = "controlDisplayName";   // 显示名称
    static const QString controlVisiblePort = "controlVisiblePort";   // 端口可见
    static const QString controlLimitingType = "controlLimitingType"; // 限幅类型
    static const QString controlVisibleType = "controlVisibleType";   // 可见类型
    static const QString controlType = "controlType";                 // 控件类型
    static const QString controlTypeValue = "controlTypeValue";       // 控件值
    static const QString controlPrompt = "controlPrompt";             // 变量说明

    static const QVariantMap controlTrMap = {{"m_controlName", "名称"}, {"m_controlValue", "变量值"}, {"m_controlDisplayName", "显示名称"}, {"m_controlVisiblePort", "端口可见"}, {"m_controlLimitingType", "限幅类型"}, {"m_controlVisibleType", "可见类型"}, {"m_controlType", "控件类型"}, {"m_controlTypeValue", "控件值"}, {"m_controlPrompt", "变量说明"}};

    static const QString controlParameters = "Parameters"; // 参数
    static const QString controlContainer = "Container";   // 容器
    static const QString controlDisplay = "Display";       // 显示
    static const QString controlOperation = "Operation";   // 操作

    static const QString propertyF = "属性";
    static const QString propertyS = "属性";
    static const QString propertyDialog = "对话框";
    static const QString propertyLayout = "布局";

    // 控件类型
    static const QString controlLineEdit = "LineEdit";
    static const QString controlComboBox = "ComboBox";
    static const QString controlCheckBox = "CheckBox";
    static const QString controlPushButton = "PushButton";
    static const QString controlGroupBox = "GroupBox";
    static const QString controlLabel = "Label";

    // 图片宽高
    static const int imageWidth = 60;
    static const int imageHeight = 40;
}

#endif
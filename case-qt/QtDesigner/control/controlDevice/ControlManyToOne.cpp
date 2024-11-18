#include "ControlManyToOne.h"

ControlManyToOne::ControlManyToOne(QObject *parent) : ControlBase(parent)
{
}

int ControlManyToOne::RTTI = ManyToOneControl;

int ControlManyToOne::rtti() const
{
    return RTTI;
}

QString ControlManyToOne::controlName()
{
    return m_controlName;
}

QString ControlManyToOne::name()
{
    return "多对一控件";
}

QString ControlManyToOne::getName()
{
    return m_name;
}

void ControlManyToOne::setName(const QString &str)
{
    if (m_name != str)
    {
        m_name = str;
    }
}

void ControlManyToOne::setControlName(QString name)
{
    if (m_controlName != name)
    {
        m_controlName = name;
    }
}

QString ControlManyToOne::controlValue()
{
    return m_controlValue;
}

void ControlManyToOne::setControlValue(QString value)
{
    if (m_controlValue != value)
    {
        m_controlValue = value;
    }
}

QString ControlManyToOne::controlDisplayName()
{
    return m_controlDisplayName;
}

void ControlManyToOne::setControlDisplayName(QString displayName)
{
    if (m_controlDisplayName != displayName)
    {
        m_controlDisplayName = displayName;
    }
}
bool ControlManyToOne::controlVisiblePort()
{
    return m_controlVisiblePort;
}

void ControlManyToOne::setControlVisiblePort(bool visiblePort)
{
    if (m_controlVisiblePort != visiblePort)
    {
        m_controlVisiblePort = visiblePort;
    }
}

ControlManyToOne::ControlLimitingType ControlManyToOne::controlLimitingType()
{
    return m_controlLimitingType;
}

void ControlManyToOne::setControlLimitingType(ControlLimitingType limitingType)
{
    if (m_controlLimitingType != limitingType)
    {
        m_controlLimitingType = limitingType;
    }
}

ControlManyToOne::ControlManyToOne::ControlVisibleType ControlManyToOne::controlVisibleType()
{
    return m_controlVisibleType;
}

void ControlManyToOne::setControlVisibleType(ControlVisibleType visibleType)
{
    if (m_controlVisibleType != visibleType)
    {
        m_controlVisibleType = visibleType;
    }
}

ControlManyToOne::ControlType ControlManyToOne::controlType()
{
    return m_controlType;
}

void ControlManyToOne::setControlType(ControlType type)
{
    if (m_controlType != type)
    {
        m_controlType = type;
    }
}

QString ControlManyToOne::controlTypeValue()
{
    return m_controlTypeValue;
}

void ControlManyToOne::setControlTypeValue(QString typeValue)
{
    if (m_controlTypeValue != typeValue)
    {
        m_controlTypeValue = typeValue;
    }
}

QString ControlManyToOne::controlPrompt()
{
    return m_controlPrompt;
}

void ControlManyToOne::setControlPrompt(QString prompt)
{
    if (m_controlPrompt != prompt)
    {
        m_controlPrompt = prompt;
    }
}

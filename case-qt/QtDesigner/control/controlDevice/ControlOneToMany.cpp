#include "ControlOneToMany.h"

ControlOneToMany::ControlOneToMany(QObject *parent) : ControlBase(parent)
{
}

int ControlOneToMany::RTTI = OneToManyControl;

int ControlOneToMany::rtti() const
{
    return RTTI;
}

QString ControlOneToMany::controlName()
{
    return m_controlName;
}

QString ControlOneToMany::name()
{
    return "一对多控件";
}

QString ControlOneToMany::getName()
{
    return m_name;
}

void ControlOneToMany::setName(const QString &str)
{
    if (m_name != str)
    {
        m_name = str;
    }
}

void ControlOneToMany::setControlName(QString name)
{
    if (m_controlName != name)
    {
        m_controlName = name;
    }
}

QString ControlOneToMany::controlValue()
{
    return m_controlValue;
}

void ControlOneToMany::setControlValue(QString value)
{
    if (m_controlValue != value)
    {
        m_controlValue = value;
    }
}

QString ControlOneToMany::controlDisplayName()
{
    return m_controlDisplayName;
}

void ControlOneToMany::setControlDisplayName(QString displayName)
{
    if (m_controlDisplayName != displayName)
    {
        m_controlDisplayName = displayName;
    }
}
bool ControlOneToMany::controlVisiblePort()
{
    return m_controlVisiblePort;
}

void ControlOneToMany::setControlVisiblePort(bool visiblePort)
{
    if (m_controlVisiblePort != visiblePort)
    {
        m_controlVisiblePort = visiblePort;
    }
}

ControlOneToMany::ControlLimitingType ControlOneToMany::controlLimitingType()
{
    return m_controlLimitingType;
}

void ControlOneToMany::setControlLimitingType(ControlLimitingType limitingType)
{
    if (m_controlLimitingType != limitingType)
    {
        m_controlLimitingType = limitingType;
    }
}

ControlOneToMany::ControlOneToMany::ControlVisibleType ControlOneToMany::controlVisibleType()
{
    return m_controlVisibleType;
}

void ControlOneToMany::setControlVisibleType(ControlVisibleType visibleType)
{
    if (m_controlVisibleType != visibleType)
    {
        m_controlVisibleType = visibleType;
    }
}

ControlOneToMany::ControlType ControlOneToMany::controlType()
{
    return m_controlType;
}

void ControlOneToMany::setControlType(ControlType type)
{
    if (m_controlType != type)
    {
        m_controlType = type;
    }
}

QString ControlOneToMany::controlTypeValue()
{
    return m_controlTypeValue;
}

void ControlOneToMany::setControlTypeValue(QString typeValue)
{
    if (m_controlTypeValue != typeValue)
    {
        m_controlTypeValue = typeValue;
    }
}

QString ControlOneToMany::controlPrompt()
{
    return m_controlPrompt;
}

void ControlOneToMany::setControlPrompt(QString prompt)
{
    if (m_controlPrompt != prompt)
    {
        m_controlPrompt = prompt;
    }
}

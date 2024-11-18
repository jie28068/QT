#include "controlBase.h"

int ControlBase::RTTI = Base;
ControlBase::ControlBase(QObject *parent) : QObject(parent)
{
}

int ControlBase::rtti() const
{
    return RTTI;
}

QString ControlBase::getName()
{
    return m_name;
}

void ControlBase::setName(const QString &str)
{
    if (m_name != str)
    {
        m_name = str;
    }
}

QString ControlBase::name()
{
    return "基础控件";
}

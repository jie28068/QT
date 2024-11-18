#ifndef CONTROLBASE_H
#define CONTROLBASE_H

#include "globalDefinition.h"

#include <QObject>

class ControlBase : public QObject
{
    Q_OBJECT
public:
    explicit ControlBase(QObject *parent = nullptr);

    enum RITT
    {
        Base = 0,
        OneToManyControl,
        ManyToOneControl
    };
    Q_ENUM(RITT)

    virtual int rtti() const;
    static int RTTI;

    virtual QString getName();
    virtual void setName(const QString &str);

    virtual QString name();

private:
    QString m_name;
};
Q_DECLARE_METATYPE(ControlBase *)
#endif
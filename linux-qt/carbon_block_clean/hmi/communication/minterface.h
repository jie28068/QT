#pragma once
#include <QString>
// 观察者
class IObserver
{
public:
    virtual void onServerTimeMessage(const QString &str) = 0;
    virtual ~IObserver() {}
};

// 主题
class ISubject
{
public:
    virtual void registerObserver(IObserver *observer) = 0;
    virtual void notifyObservers(const QString &str) = 0;
    virtual ~ISubject() {}
};
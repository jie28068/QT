#pragma once

#include "grpcServer.h"
#include "grpcClinet.h"

#include <QThread>

#include <QDebug>
class SchedulingServiceImpl;
class ServerThread : public QThread
{
    Q_OBJECT
public:
    ServerThread(QSharedPointer<SchedulingServiceImpl> service, QObject *parent = nullptr);

    void run() override;

    ~ServerThread() override;

private:
    QSharedPointer<SchedulingServiceImpl> m_service;
};

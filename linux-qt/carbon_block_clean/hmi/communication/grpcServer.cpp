#include "grpcServer.h"
#include <QDebug>

SchedulingServiceImpl::SchedulingServiceImpl()
{
}

grpc::Status SchedulingServiceImpl::SystemStatus(grpc::ServerContext *context, const SystemStatusReq *request, SystemStatusResp *response)
{
    // std::cout << "SystemStatus  with json_info: " << request->task() << request->json_info() << std::endl;
    QString json_info = QString::fromStdString(request->json_info());
    notifyObservers(json_info);
    return Status::OK;
}

void SchedulingServiceImpl::registerObserver(IObserver *observer)
{
    std::lock_guard<std::mutex> lock(observers_mutex); // 加锁
    observers.push_back(observer);
}

void SchedulingServiceImpl::notifyObservers(const QString &str)
{
    std::lock_guard<std::mutex> lock(observers_mutex); // 加锁
    for (IObserver *observer : observers)
    {
        observer->onServerTimeMessage(str);
    }
}

SchedulingLogSend::SchedulingLogSend()
{
}

grpc::Status SchedulingLogSend::LogInfo(grpc::ServerContext *context, const LogReq *request, LogResp *response)
{
    return grpc::Status();
}

void SchedulingLogSend::registerObserver(IObserver *observer)
{
}

void SchedulingLogSend::notifyObservers(const QString &str)
{
}

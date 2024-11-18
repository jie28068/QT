#pragma once

#include <memory>
#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/status.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>

#include "minterface.h"
#include "msg/scheduling.grpc.pb.h"
#include "msg/scheduling.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using Scheduling::StatusSend;
using Scheduling::SystemStatusReq;
using Scheduling::SystemStatusResp;

using Scheduling::LogReq;
using Scheduling::LogResp;
using Scheduling::LogSend;

class SchedulingServiceImpl final : public Scheduling::StatusSend::Service, public ISubject
{
public:
    SchedulingServiceImpl();
    grpc::Status SystemStatus(grpc::ServerContext *context, const SystemStatusReq *request,
                              SystemStatusResp *response);

public:
    void registerObserver(IObserver *observer) override;
    void notifyObservers(const QString &str) override;

private:
    std::vector<IObserver *> observers;
    std::mutex observers_mutex; // 添加互斥锁
};

class SchedulingLogSend final : public Scheduling::LogSend::Service, public ISubject
{
public:
    SchedulingLogSend();
    grpc::Status LogInfo(grpc::ServerContext *context, const LogReq *request,
                         LogResp *response);

public:
    void registerObserver(IObserver *observer) override;
    void notifyObservers(const QString &str) override;

private:
    std::vector<IObserver *> observers;
    std::mutex observers_mutex; // 添加互斥锁
};
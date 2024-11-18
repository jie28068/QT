#pragma once

#include <memory>
#include <grpcpp/grpcpp.h>

#include "msg/hostCompute.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using HostCompute::HostCompute;
using HostCompute::SchedulingRequest;
using HostCompute::SchedulingResponse;

class HostComputeClient
{
public:
    HostComputeClient(std::shared_ptr<Channel> channel);

    std::pair<bool, std::string> ScheduleTask(int32_t task, const std::string &cardata = "");

private:
    std::unique_ptr<HostCompute::HostCompute::Stub> stub_;
};
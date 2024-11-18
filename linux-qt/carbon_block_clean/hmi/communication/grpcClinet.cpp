#include "grpcClinet.h"
#include "define.h"
#include "glog/logging.h"

#include <QDebug>
#include <fstream>
#include <utility>
#include <sstream>
HostComputeClient::HostComputeClient(std::shared_ptr<Channel> channel)
    : stub_(HostCompute::HostCompute::NewStub(channel))
{
}

std::pair<bool, std::string> HostComputeClient::ScheduleTask(int32_t task, const std::string &cardata)
{
    SchedulingRequest request;
    request.set_task(task);
    request.set_carbon_data(cardata);

    SchedulingResponse response;
    ClientContext context;

    // 设置请求超时时间为3秒
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(3);
    context.set_deadline(deadline);
    Status status = stub_->Scheduling(&context, request, &response);

    std::pair<bool, std::string> result;
    if (status.ok())
    {
        if (response.abnormal() == "abnormal")
        {
            result.first = false;
        }
        else if (response.abnormal() == "normal")
        {
            result.first = true;
        }
        result.second = response.json_info();
    }
    else
    {
        result.first = false;
        result.second = "";
    }
    return result;
}

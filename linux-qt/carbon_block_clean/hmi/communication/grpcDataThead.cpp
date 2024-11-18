#include "grpcDataThead.h"
#include <QEventLoop>
ServerThread::ServerThread(QSharedPointer<SchedulingServiceImpl> service, QObject *parent)
    : QThread(parent), m_service(service)
{
}

void ServerThread::run()
{
    std::string server_address("0.0.0.0:50055");
    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(m_service.data());
    std::unique_ptr<Server> server(builder.BuildAndStart());
    // 事件循环，直到被告知退出
    exec();
    // 关闭服务器
    server->Shutdown();
    // 等待服务器完成所有挂起的调用
    server->Wait();
}

ServerThread::~ServerThread()
{
}

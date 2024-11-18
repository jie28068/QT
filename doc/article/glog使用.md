[TOC]

### 概述

- 在Ubuntu下使用glog是一个非常流行的日志库，它提供了C++程序中灵活的日志记录功能。

### 安装

```shell
sudo apt-get install libunwind-dev
sudo apt-get install libgoogle-glog-dev
```

![alt text](image.png)

```cpp
#include <glog/logging.h>
#include <sys/stat.h>
#include <sys/types.h>

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler(); // 自动记录当前的堆栈跟踪到日志文件中
  FLAGS_alsologtostderr = true;          // 允许你在控制台看到日志信息
  FLAGS_colorlogtostderr = true;         // 输出到标准错误时带有颜色
  FLAGS_logbuflevel = google::INFO;
  mkdir("../log", S_IRWXU | S_IRWXG | S_IRWXO);
  FLAGS_log_dir = "../log";

  LOG(INFO) << "这是一个信息日志";
  LOG(WARNING) << "这是一个警告日志";
  LOG(ERROR) << "这是一个错误日志";

  google::ShutdownGoogleLogging(); // 退出glog
  return 0;
}
```

```cpp
cmake_minimum_required(VERSION 3.0)
project(test_pg)
set(CMAKE_CXX_STANDARD 11)

add_executable(test_pg src/pgtest.cc)
target_link_libraries(test_pg glog)
```

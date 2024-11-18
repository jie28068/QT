#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// void func(std_msgs::String msg)
// {
//     ROS_INFO(msg.data.c_str());
// }
// void funcx(std_msgs::String msg)
// {
//     ROS_WARN(msg.data.c_str());
// }
// void funcxx(std_msgs::String msg)
// {
//     ROS_WARN(msg.data.c_str());
// }
// int main(int argc, char **argv)
// {
//     setlocale(LC_ALL, ""); // 中文 解决接受显示乱码问题
//     ros::init(argc, argv, "na_node");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe("/hmi/control_command", 10, func);
//     ros::Subscriber sub_2 = nh.subscribe("/hmi/robot_control", 10, funcx);
//     ros::Subscriber sub_3 = nh.subscribe("/hmi/task_enable", 10, funcxx);
//     while (ros::ok())
//     {
//         ros::spinOnce(); // 持续获取
//         /* code */
//     }
// }

#include "HmiStatus.h"
class MySubscriberNode
{
public:
    MySubscriberNode(ros::NodeHandle *node_ptr) : node_ptr_(node_ptr)
    {
        sub_ = node_ptr_->subscribe("/hmi/task_enable", 1, &MySubscriberNode::taskEnableCallback, this);
    }

    ~MySubscriberNode()
    {
        sub_.shutdown();
    }

private:
    void taskEnableCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        ROS_INFO("Received task enable message: %d", msg->data);
    }

    ros::NodeHandle *node_ptr_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "na_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    // // 创建订阅者节点实例
    // MySubscriberNode subscriberNode(&nh);

    // // 处理回调函数
    // ros::spin();

    ///-------------------------------------------
    // 创建发布者
    ros::Publisher pub = nh.advertise<hmi_qt::HmiStatus>("/hmi_messages", 1);

    // 创建消息
    hmi_qt::HmiStatus msg;

    char choice;

    do
    {
        // 从终端读取hmi_robot_status的值
        std::cout << "请输入hmi_robot_status的值: ";
        std::cin >> msg.hmi_robot_status;

        // 从终端读取hmi_plc_status的值
        std::cout << "请输入hmi_plc_status的值: ";
        std::cin >> msg.hmi_plc_status;

        // 清除输入缓冲区中的任何剩余字符，包括换行符
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // 等待直到有订阅者连接
        while (pub.getNumSubscribers() == 0)
        {
            ROS_WARN_ONCE("请创建一个订阅者到/hmi_messages主题");
            ros::Duration(0.5).sleep(); // 等待0.5秒
        }

        // 发布消息
        pub.publish(msg);
        ROS_INFO("消息已发送");

        // 等待一段时间以确保消息被发送
        ros::spinOnce();

        // 询问用户是否想要继续发送
        std::cout << "是否要发送另一条消息? (y/n): ";
        std::cin >> choice;

        // 清除输入缓冲区中的任何剩余字符，包括换行符
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    } while (choice == 'y' || choice == 'Y');
    return 0;
}

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "zha_node");
    // 句柄
    ros::NodeHandle nh;
    // 创建发布者 注意后面订阅者需要与"zzz"话题一直
    ros::Publisher pub = nh.advertise<std_msgs::String>("zzz", 10);
    // 循环发布
    while (ros::ok())
    {
        printf("你好\n");
        // 创建消息并发布
        std_msgs::String msg;
        msg.data = "鸡你太美";
        pub.publish(msg);
    }
    return 0;
}
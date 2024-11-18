
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  // 初始化节点
  ros::init(argc, argv, "chao_node");
  // 句柄
  ros::NodeHandle nh;
  // 创建发布者 注意后面订阅者需要与"wahaha"话题一直
  ros::Publisher pub = nh.advertise<std_msgs::String>("wahaha", 10);
  // 循环发布
  while (ros::ok())
  {
    printf("wozai\n");
    // 创建消息并发布
    std_msgs::String msg;
    msg.data = "人在塔在";
    pub.publish(msg);
  }
  return 0;
}
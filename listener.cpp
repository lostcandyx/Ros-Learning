#include "ros/ros.h"
#include "std_msgs/String.h"
/*
 *这是一个回调函数，当接收到 chatter 话题的时候就会被调用。
 *消息是以 boost shared_ptr 指针的形式传输，这就意味着你可以存储它而又不需要复制数据。
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // 告诉 master 我们要订阅 chatter 话题上的消息。当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数。
  // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1000 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
  // NodeHandle::subscribe() 返回 ros::Subscriber 对象,你必须让它处于活动状态直到你不再想订阅该消息。
  // 当这个对象销毁时，它将自动退订 chatter 话题的消息。
  
  ros::spin();

  return 0;
}
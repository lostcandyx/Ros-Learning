#include "ros/ros.h"
//ros/ros.h 是一个实用的头文件，它引用了 ROS 系统中大部分常用的头文件。
#include "std_msgs/String.h"
//这引用了 std_msgs/String 消息, 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件。
//需要关于消息的定义，可以参考 msg 页面。
#include <sstream>

/**
 * http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  //初始化 ROS 。它允许 ROS 通过命令行进行名称重映射——然而这并不是现在讨论的重点。
  //在这里，我们也可以指定节点的名称——运行过程中，节点的名称必须唯一。这里的名称必须是一个 base name ，也就是说，名称内不能包含 / 等符号。

  ros::NodeHandle n;
  //为这个进程的节点创建一个句柄。第一个创建的 NodeHandle 会为节点进行初始化，最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源。

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息。
  //这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。
  //第二个参数是发布序列的大小。如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。
  //NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用： 
  //1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 
  //2) 如果消息类型不对,它会拒绝发布。

  ros::Rate loop_rate(10);
  //ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。
  
  int count = 0;
  /*
   *关于ros::ok()
   *roscpp 会默认生成一个 SIGINT 句柄，它负责处理 Ctrl-C 键盘操作——使得 ros::ok() 返回 false。
   *如果下列条件之一发生，ros::ok() 返回false：

    SIGINT 被触发 (Ctrl-C)
    被另一同名节点踢出 ROS 网络
    ros::shutdown() 被程序的另一部分调用
    节点中的所有 ros::NodeHandles 都已经被销毁

    一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
   */
  while (ros::ok())
  {
    std_msgs::String msg;
    //我们使用一个由 msg file 文件产生的『消息自适应』类在 ROS 网络中广播消息。现在我们使用标准的String消息，它只有一个数据成员 "data"。

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    //ROS_INFO 和其他类似的函数可以用来代替 printf/cout 等函数。
    chatter_pub.publish(msg);
    //我们向所有订阅 chatter 话题的节点发送消息。
    ros::spinOnce();

    loop_rate.sleep();
    //这条语句是调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz。
    ++count;
  }


  return 0;
}
# Ros-Learning 的笔记
## 文件系统概念
- `Packages 软件包`:是ROS应用程序代码的组织单元，每个软件包都可以包含程序库、可执行文件、脚本或者其它手动创建的东西。

- `Manifest (package.xml) 清单`:是对于'软件包'相关信息的描述,用于定义软件包相关元信息之间的依赖关系，这些信息包括版本、维护者和许可协议等。

## 控制台指令
```shell
$ rospack find [包名称]
# 返回软件包的路径信息
```
```shell
$ roscd [包名称]
# 直接切换（cd）工作目录到某个软件包或者软件包集当中
$ pwd
# 输出当前的目录
$ roscd log
# 切换到 ROS 保存日记文件的目录下
```
```shell
$ rosls [本地包名称[/子目录]]
# 返回软件包的路径信息
```
## 创建ROS程序包
catkin程序包中至少要有两个文件，`CMakeLists.txt`,`package.xml`，catkin工作空间的结构如下：
```shell
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```
使用以下指令来创建一个catkin工作空间：
```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
使用catkin_create_pkg命令来创建一个程序包：
```shell
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# 新程序包名为'beginner_tutorials'的，这个程序包依赖于std_msgs、roscpp和rospy，指令的具体形式为：
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
rospack列出了在运行catkin_create_pkg命令时作为参数的依赖包，这些依赖包随后保存在package.xml文件中，同样对于依赖包来说其本身可能还有依赖包，也可以使用这个指令来查看依赖项。
```shell
rospack depends1 [依赖包名称]
rospack depends1 [程序包名称]
# 以上只会检测第一层依赖
rospack depends [包名称]
# 使用‘depneds’会递归检测所有依赖
```
在package.xml中需要编辑很多标签，这些不是必须的工作，在笔记中不再赘述，教程网址：
[自定义 package.xml](http://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage)

## 编译ROS程序包
catkin_make 是一个命令行工具，它简化了catkin的标准工作流程。你可以认为catkin_make是在CMake标准工作流程中依次调用了cmake 和 make。使用方法:
```shell
# 在catkin工作空间下
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```
CMake标准工作流程如下：
```shell
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  #(可选)
```
## 一些概念概述

|   Name    | discription |
| :---:     | :--- |
| Nodes     | 节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。 |
| Messages  | 消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。 |
| Topics    | 话题,节点可以发布消息到话题，也可以订阅话题以接收消息。 |
| Master    | 节点管理器，ROS名称服务 (比如帮助节点找到彼此)。 |
| rosout    | ROS中相当于stdout/stderr,专用的输入输出 |
| roscore   | 主机  + rosout + 参数服务器 的总称 |

## 节点
一个节点其实只不过是ROS程序包中的一个可执行文件。ROS节点可以使用ROS客户库与其他节点通信。节点可以发布或接收一个话题。节点也可以提供或使用某种服务。
`roscore` 是你在运行所有ROS程序前首先要运行的命令。
运行了roscore以后，在新的终端中可以使用`rosnode`现实当前运行的ROS节点信息：
```shell
$ rosnode list
# 指令列出活跃的节点
# 实验中出现的是rosout节点，这个节点用于信息的收集和记录节点调试输出信息，所以它总是在运行的。
$ rosnode info /rosout 
# 查看特定节点的信息
$ rosrun [package_name] [node_name]
# 使用包名直接运行一个包内的节点，使用这个指令不需要包的路径
# 这个指令的延伸还可以更改节点在list中的名字（重新配置名称）。
$ rosnode ping [node_name]
# 这里的node_name和ros run指令中的不一样，是list列出的名字，是可以更改的
$ rosnode cleanup
# 清除节点，在我们使用ctrl + C关闭节点程序而不是直接关掉窗口，则list中仍然会有显示
# 这时候使用这个指令就可以清除这个节点程序
```
## 话题
节点之间的通讯是通过一个**话题**（topic）实现的，以小乌龟为例：
turtle_teleop_key在一个话题上**发布**按键输入消息，而turtlesim则**订阅**该话题以接收该消息。

rqt_graph能够创建一个显示当前系统运行情况的动态图形，运行该节点的指令如下：
```shell
rosrun rqt_graph rqt_graph
```
通过鼠标激活高亮可以查看系统的运行情况，通过箭头的方向可以得知发布方和订阅方
```shell
$ rostopic -h
# 查看rostopic的所有子命令
$ rostopic echo [topic]
# 显示在某个话题上发布的数据
$ rostopic list
# 列出所有当前订阅和发布的话题。
$ rostopic list -h
# 查看rostopic list指令的帮助
$ rostopic list -v
# 显示出有关所发布和订阅的话题及其类型的详细信息
$ rostopic pub [topic] [msg_type] [args]
# 把数据发布到当前某个正在广播的话题上
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
# -1 这个参数选项使rostopic发布一条消息后马上退出
# -- 这个参数会告诉命令选项解析器接下来的参数部分都不是命令选项。
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
# 这条命令以1Hz的频率发布速度命令到速度话题上，-r表示发送稳定的命令流，1应该表示的是1Hz的频率
$ rostopic hz [topic]
# 查看消息发布的频率
```
这里记录以下教程开启的流程：
```shell
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key
$ rosrun rqt_graph rqt_graph
```
## 消息
话题之间的通信是通过在节点之间发送ROS消息实现的。对于发布器和订阅器之间的通信，发布器和订阅器之间必须发送和接收相同类型的消息。这意味着话题的类型是由发布在它上面的消息类型决定的。
```shell
$ rostopic type [topic]
# 查看所发布话题的消息类型
$ rosmsg show [message type]
# 查看消息的详细情况
```
rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形，很类似于串口监视器：
```shell
$ rosrun rqt_plot rqt_plot
```

## 服务
rosservice可以很轻松的使用 ROS 客户端/服务器框架提供的服务。指令和话题的有些类似：
```shell
$ rosservice list
# 罗列提供的服务
$ rosservice type [service]
# 查看某一个服务的类型，服务的类型为空（empty),这表明在调用这个服务是不需要参数
$ rosservice call [service] [args]
# 使用命令调用服务
```

## 参数
rosparam使得我们能够存储并操作ROS 参数服务器（Parameter Server）上的数据。参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型。rosparam使用YAML标记语言的语法。一般而言，YAML的表述很自然：1 是整型, 1.0 是浮点型, one是字符串, true是布尔, [1, 2, 3]是整型列表, {a: b, c: d}是字典。 rosparam用来操作参数的指令如下：
```shell
$ rosparam set [param_name] [value]           
# 设置参数，设置成功之后需要调用清除服务来使得修改后的参数生效。
$ rosparam get [param_name]           
# 获取参数
$ rosparam load [file_name] [namespace]       
# 从文件读取参数
$ rosparam dump [file_name]         
# 向文件中写入参数
$ rosparam delete         
# 删除参数
$ rosparam list
# 列出参数名
```

## 控制台与日志
rqt_console属于ROS日志框架(logging framework)的一部分，用来显示节点的输出信息。rqt_logger_level允许我们修改节点运行时输出信息的日志等级（logger levels）（包括 DEBUG、WARN、INFO和ERROR）。
```shell
$ rosrun rqt_console rqt_console
# 运行rqt_console
$ rosrun rqt_logger_level rqt_logger_level
# 运行rqt_logger_level
```
日志等级按以下优先顺序排列：
| Fatal | Error | Warn | Info | Debug |
| --- | --- | --- | --- | --- |

Fatal是最高优先级，Debug是最低优先级。通过设置日志等级你可以获取该等级及其以上优先等级的所有日志消息。比如，将日志等级设为Warn时，你会得到Warn、Error和Fatal这三个等级的所有日志消息。
roslaunch可以用来启动定义在launch文件中的多个节点。
```shell
$ roslaunch [package] [filename.launch]
# 这个指令可以和launch文件进行互动
```
以下是launch文件的信息：
以launch标签开头以表明这是一个launch文件，标签的写法是&lt;launch&gt;，使用尖括号，结束标签为&lt;/launch&gt;。
在launch文件中可以使用命名空间，简写为ns。
[详细讲解](http://wiki.ros.org/cn/ROS/Tutorials/UsingRqtconsoleRoslaunch#CA-91a3946a9c4cf7301bb55ec0c3f8a77f6c8f9777_1)

## 编辑ROS中的文件
rosed 是 rosbash 的一部分。利用它可以直接通过package名来获取到待编辑的文件而无需指定该文件的存储路径了。
```shell
$ rosed [package_name] [filename]
# 默认使用vim编辑器编辑文件
$ export EDITOR='emacs -nw'
# 设置emacs为默认编辑器
```

## 创建ROS消息和ROS服务
- 消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。放在package的msg目录下。
- 服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。存放在srv目录下。
msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下：
- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]



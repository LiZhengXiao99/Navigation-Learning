[TOC]

## 基本操作

|                操作                |                             命令                             |
| :--------------------------------: | :----------------------------------------------------------: |
|           启动节点管理器           |                           roscore                            |
|         ⼀次性启动多个节点         |                          roslaunch                           |
|              启动节点              |           rosrun [package-name] [executable-name]            |
|          获得运行节点列表          |                         rosnode list                         |
| 使用 rosrun 命令显式设置节点的名称 | rosrun [package-name] [executable-name] __name:=[node-name]  |
|        安装 turtlesim 功能         |            sudo apt install ros-kinetic-turtlesim            |
|           启动turtlesim            | roscore、rosrun turtlesim turtlesim_node、rosrun turtlesim turtle_teleop_key |
|     查看软件包列表和定位软件包     |                         rospack list                         |
|        找到⼀个软件包的目录        |                 rospack find [package-name]                  |
|           查看包的依赖项           |              rsopack depends tur turtlesim_node              |
|       查看软件包目录下的文件       |                     rosls [package-name]                     |
|    将当前目录切换到此软件包目录    |                     roscd [package-name]                     |

## 节点操作

|          操作           |                         命令                         |
| :---------------------: | :--------------------------------------------------: |
| 查看所有正在运⾏的 Node |                     rosnode list                     |
|        运⾏ Node        | rosrun [package_name] [node_name] [__name:=new_name] |
|      查看节点信息       |               rosnode info [node-name]               |
|        终⽌节点         |               rosnode kill [node-name]               |
|   将节点从列表中删除    |                   rosnode cleanup                    |
| 查看节点之间的连接关系  |                      rqt_graph                       |

## 话题操作

|             操作             |                             命令                             |
| :--------------------------: | :----------------------------------------------------------: |
|      列出当前活跃的话题      |                        rostopic list                         |
|   查看某个话题上发布的消息   |                   rostopic echo topic-name                   |
|       查看话题消息格式       |        rostopic type [topic]、rosmsg show [msg_type]         |
|    输出每秒发布的消息数量    |                   rostopic hz [topic-name]                   |
| 输出每秒发布消息所占的字节量 |                   rostopic bw [topic-name]                   |
|    获取更多关于话题的信息    |                  rostopic info [topic-name]                  |
|        查找特定的节点        |                 rostopic list \| grep [XXX]                  |
|         查看消息类型         |               rosmsg show [message-type-name]                |
|         发布制定消息         | rostopic pub –r rate-in-hz [topic-name] [message-type] [message-content] |

## 编译操作

|                  操作                  |                            命令                             |
| :------------------------------------: | :---------------------------------------------------------: |
|              编译指定的包              | catkin_make -DCATKIN_WHITELIST_PACKAGES=”package1;package2″ |
|            恢复编译所有的包            |         catkin_make -DCATKIN_WHITELIST_PACKAGES=””          |
|       提取⽂件系统上的功能包信息       |                           rospack                           |
| ⽤于⽂件系统上的功能包集信息的命令⼯具 |                          rosstack                           |
|           查找到某个功能包集           |                     rosstack find [xxx]                     |

## 常用命令

|              操作              |                             命令                             |
| :----------------------------: | :----------------------------------------------------------: |
| 显示消息或者服务的数据结构定义 |                        rosmsg/rossrv                         |
|       显示消息中域的定义       |                         rosmsg show                          |
|     显⽰调⽤指定消息的代码     |                         rosmsg users                         |
|  列出指定功能包中的所有的消息  |                        rosmsg package                        |
|   列出带有该消息的所有功能包   |                       rosnode packages                       |
|     显⽰关于节点的调⽤信息     |                        rosnode [xxx]                         |
|    测试到⼀个节点的可连接性    |                      rosnode ping [xxx]                      |
|        列出所有活动节点        |                      rosnode list [xxx]                      |
|          列出节点信息          |                      rosnod info [xxx]                       |
|         结束运行的节点         |                      rosnod kill [xxx]                       |
|          结束所有节点          |                       rosnode kill -a                        |
|       在功能包内启动⽂件       |             roslaunch package [filename.launch]              |
|       在局部节点启动⽂件       |          roslaunch –local package [filename.launch]          |
|     安装⼀个包的系统依赖项     |                   rosdep [filename.launch]                   |
|  以图形界⾯显示⼀个包的依赖项  |                           rqt_dep                            |
|            编辑⽂件            |                            rosed                             |
|       从⼀个包中复制文件       |                            roscp                             |
|        列出⼀个包的目录        |                             rosd                             |
|          加载参数文件          |                      rosparam load file                      |
|         输出参数到文件         | rosparam dump file、rosparam delete parameter、rosparam list |

## 图形界面操作

|              操作              |                      命令                      |
| :----------------------------: | :--------------------------------------------: |
| 以界⾯的形式显示正在运行的节点 |             rosrun rqt_top rqt_top             |
|  界⾯的形式显⽰话题的调试信息  |                   rqt_topic                    |
|   界⾯的形式显⽰订阅者的信息   |                 rqt_publisher                  |
|     以图形界⾯调⽤服务信息     |               rqt_service_calle                |
|       查看节点发出的消息       |                  rqt_console                   |
|          设置动态参数          |                rqt_reconfigure                 |
|            节点监测            | rosrun rqt_runtime_monitor rqt_runtime_monitor |
|        监测所有节点信息        |   rosrun rqt_robot_monitor rqt_robot_monitor   |
|      绘制某个消息的曲线图      |         rosrun rqt_plot rqt_plot [xxx]         |
|        显⽰三个消息参数        |    rosrun rqt_plot rqt_plot [/accel/x:y:z]     |
|            查看图片            |      rosrun rqt_image_view rqt_image_view      |
|   以图形界⾯查看包的内部信息   |                    rqt_bag                     |

## Bag 操作

|        操作         |                   命令                    |
| :-----------------: | :---------------------------------------: |
| 记录所有 topic 变化 |             rosbag record -a              |
|   记录某些 topic    | rosbag record -O subset [topic1] [topic2] |
|    查看 bag 信息    |        rosbag info [bagfile_name]         |
|        回放         |     rosbag play (-r 2) [bagfile_name]     |

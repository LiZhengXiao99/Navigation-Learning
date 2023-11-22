[TOC]

## 一、nav_core

这个文件夹里只有 4 个头文件，定义了 4 个，所有的路径规划与恢复行为都使⽤插件形式继承这些接口

* **base_global_planner.h**：
* **base_local_planner.h**：
* **parameter_magic.h**：
* **recovery_behavior.h**：





导航包里有一个 nav_core，是一个纯粹的接口，里面规定了核心的几个类应该有的样子。BaseGlobalPlanner 是全局导航的接口，规定了一个功能makePlan，也就是给定起始跟目标，输出一系列pose走过去；BaseLocalPlanner 规定了一个核心功能 computeVelocityCommands，就是计算局部地图下一步的控制指令；还有一个 RecoveryBehavior，规定一个 runBehavior，也就是小车卡住情况下如何运动恢复到正常的导航。





## 二、move_base

movebase是navigation中的核心，实现了从机器人起始位置到目标位置的路径规划和运动控制的功能。 它融合了如下的接口，作为插件被 move_base 调用：

* **global_planner**： 实现路径规划
* **local_planner**： 实现轨迹规划和速度控制
* **local_costmap**：用于避障
* **global_costmap**：用于路径规划
* **recovery**：导航过程中的异常处理

![image-20231111160631063](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231111160631063.png)

move_base 文件结构很简单：

```python
move_base/					
├── CHANGELOG.rst
├── CMakeLists.txt
├── cfg
│   └── MoveBase.cfg
├── include
│   └── move_base
│       └── move_base.h
├── package.xml
├── planner_test.xml
└── src
    ├── move_base.cpp
    └── move_base_node.cpp
```

用 cloc 统计代码量如下：

![image-20231105102346754](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231105102346754.png)





MoveBase构造函数中：

* 

* **订阅的话题**：

  ```cpp
  ros::NodeHandle simple_nh("move_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
  boost::bind(&MoveBase::goalCB, this, _1));
  ```

* **发布的话题**：

  ```cpp
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>
  ("current_goal", 0 );
  action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>
  ("goal", 1);
  recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>
  ("recovery_status", 1);
  ```

* **提供的 service**：

  ```cpp
  make_plan_srv_ = private_nh.advertiseService("make_plan",
  &MoveBase::planService, this);
  clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps",
  &MoveBase::clearCostmapsService, this);
  ```


![image-20231112215041090](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231112215041090.png)



**movebase** 状态之间切换条件：

* **切换到 CLEARING**：
  * 得到路径但计算不出下一步控制时重新进行路径规划
  *  没有超时，但是没有找到有效全局路径
* **切换到 CONTROLLING**：
  * 获得了全局路径，并且没有到达目标点
* **切换到 PLANNING**：
  * 构造函数初始化
  * 获得新的目标点
  * 目标点的坐标系和全局坐标系不一致
  * 执行 recovery 之后
  * resetState





在导航模块出现异常的时候，恢复行为会被触发。其中全局路径规划，局部规划或者来回震荡都会导致 recovery behavior 被触发。代码逻辑中是 move_base.cpp 中的 state 会被更新为*CLEARING*状态，*recovery_trigger* 也会被更新为相应的行为。

恢复行为：

* PLANNING_R：全局规划失败
* CONTROLLING_R：局部轨迹规划失败
* OSCILLATION_R：长时间在小区域运动

navigation中提供了三个恢复的行为：

* clear_costmap_recovery
* rotate_recovery
* move_slow_and_clear

loadDefaultRecoveryBehaviors中依次加载了 conservative_reset rotate_recovery aggressive_reset rotate_recovery：

![image-20231111213829651](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231111213829651.png)






























[TOC]





## 条件变量

在多线程编程中，有一种十分常见的行为：线程同步。线程同步是指线程间需要按照预定的先后次序顺序进行的行为。C++11对这种行为也提供了有力的支持，这就是条件变量(condition_variable和condition_variable_any)。条件变量位于头文件condition_variable下。

condition_variable/condition_variable_any类是一个synchronization primitive，可用于阻止一个线程或同时阻止多个线程，直到另一个线程修改共享变量（condition），并通知condition_variable，才会继续执行。

当调用它的wait函数时，它使用一个mutex来锁定线程。使得该线程保持阻塞状态，直到被另一个线程调用同一个condition_variable对象上的notify函数才被唤醒。condition_variable类型的对象必须使用`unique_lock<mutex>`等待，而 std::condition_variable_any可以跟任何其他可锁定对象绑定使用, 也可以使用自定义类型。



## 加强版的 mutex

- `std::mutex`: 独占的互斥量，不能递归使用
- `std::recursive_mutex`: 递归互斥量，不带超时功能。C++11考虑到同一线程重复使用互斥量的需求，提供了`std::recursive_mutex`允许互斥量的重复进入，但要注意释放问题，进入几次就需要释放几次。
- `std::timed_mutex`: 带超时的独占互斥量，不能递归使用。std::timed_mutex设置了等待超时的机制，之前的互斥量如果无法等待进入机会，会一直阻塞线程，使用std::timed_mutex可以为锁的等待设置一个超时值，一旦超时可以做其他事情。
  * `try_lock` 避免了阻塞，获取不到锁则直接返回false
  * `try_lock_for()` 尝试锁定互斥，若互斥在指定的时限时期中不可用则返回 `等待一段时间`
  * `try_lock_until()` 尝试锁定互斥，直至抵达指定时间点互斥不可用则返回 `一个时间点`
- `std::recursive_timed_mutex`: 带超时的递归互斥量，结合了超时和递归



## Actionlib

在任何大型的基于ROS的系统中，都有这样的情况:有人想向某个节点发送请求，以执行某些任务，并接收对请求的应答。这可以通过ROS服务来实现。但是，在某些情况下，如果服务需要很长时间执行，用户可能希望在执行过程中取消请求，或者得到关于请求进展情况的定期反馈。actionlib包提供了创建服务器的工具，这些服务器执行可被抢占的长期目标。它还提供了一个客户端接口，以便向服务器发送请求。

actionlib接口不但可以调度任务的执行，而且具备中断任务、任务状态跟踪与周期性状态反馈、执行过程中间结果反馈的能力。

actionlib接口定义了ActionClient(任务请求的客户端)与ActionServer(任务调度的服务器端)。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20181013161951725)

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20161109133711682)



**Goal**：在移动底盘的情况下，目标将是一个包含关于机器人应该移动到世界何处的信息的、具有固定功能的信息。为了控制倾斜激光扫描仪，目标将包含扫描参数(最小角度，最大角度，速度等)。

**Feedback**：反馈为服务器实现者提供了一种方式来告诉一个ActionClient关于一个目标的渐进进展。对于移动底盘，这可能是机器人在路径上的当前姿态。为了控制倾斜的激光扫描仪，这可能是在扫描完成之前的时间。

**Result**：在完成目标后，将结果从ActionServer发送到ActionClient。这与反馈不同，因为它只发送一次。当行动的目的是提供某种信息时，这是非常有用的。对于移动底盘，其结果并不十分重要，但它可能包含机器人的最终姿态。为了控制倾斜的激光扫描仪，结果可能包含从所请求的扫描产生的点云。



## geometry_msgs：几何学数据类型

geometry_msgs意思是几何学数据类型，是ROS元功能包common_msgs中提供的许多不同消息类型中的一种。支持的消息类型包括：

Accel、AccelStamped、AccelWithCovariance、AccelWithCovarianceStamped、Inertia、InertiaStamped、Point、Point32、PointStamped、Polygon、PolygonStamped、Pose、Pose2D、PoseArray、PoseStamped、PoseWithCovariance、PoseWithCovarianceStamped、Quaternion、QuaternionStamped、Transform、TransformStamped、Twist、TwistStamped、TwistWithCovariance、TwistWithCovarianceStamped、Vector3、Vector3Stamped、Wrench、WrenchStamped

常用的消息数据类型具体内容：

#### Point：三维 float64 点

```cpp
//一般情况下推荐使用point类型，也就是64位的定义方式，可以促进互操作性
（可能ros中大多数函数的接口是用64位定义的）。
float64 x
float64 y
float64 z
```

#### Point32：三维 float32 点

```cpp
//一般使用Point，大规模点云使用Point32，可以减少所需的内存空间，
//例如发布点云数据的时候。 
float32 x
float32 y
float32 z 
```

#### PointStamped：包含坐标系和时间戳信息

```cpp
std_msgs/Header header // 包含坐标系和时间戳信息  
    uint32 seq #存储原始数据类型
    time stamp #存储ROS中的时间戳信息
    string frame_id #用于表示和此数据关联的帧，在坐标系变化中可以理解为数据所在的坐标系名称
geometry_msgs/Point point
```

#### Quaternion：四元数

```cpp
float64 x
float64 y
float64 z
float64 w
```

#### Vector3：三维 float32 向量

```cpp
float32 x
float32 y
float32 z 
```

#### Twist：控制指令

```cpp
geometry_msgs/Vector3 linear //线速度
geometry_msgs/Vector3 angular //角速度
```

#### Pose：位姿，Point + Quarternion

```cpp
geometry_msgs/Point position # 调用的上面的Point
	float64 x
	float64 y
	float64 z
geometry_msgs/Quaternion orientation #调用的Quarternion
	float64 x
	float64 y
	float64 z
	float64 w
```

#### Pose2D：2D平面的位姿

```cpp
float64 x
float64 y
float64 theta
```

#### PoseArray：位姿序列

```cpp
std_mags/Header header 
geometry_msgs/Pose[] poses
```

#### PoseStamped

```cpp
std_msgs/Header header  
geometry_msgs/Pose pose
```

#### PoseWithCovariance：带协方差的位姿

```spp
＃这表示具有不确定性的自由空间中的姿势。
＃6x6协方差矩阵的行主要表示
＃方向参数使用固定轴表示。
＃按顺序，参数为：
# (x，y，z，绕X轴旋转，绕Y轴旋转，绕Z轴旋转）
 
geometry_msgs/Pose pose #这个又用的是Pose的消格式
float64[36] covariance #表示协方差
```

#### TransformStamped：表示两个坐标系之间的转换信息

```spp
//这表示从坐标框架header.frame_id到坐标框架child_frame_id的转换此消息主要由tf软件包使用。
std_mags/Header header 
    uint32 seq #存储原始数据类型
    time stamp #存储ROS中的时间戳信息
    string frame_id #用于表示和此数据关联的帧，在坐标系变化中可以理解为数据所在的坐标系名称
 
string child_frame_id   # the frame id of the child frame
 
geometry_msgs/Transform transform //调用
    geometry_msgs/Vector3 translation # 平移向量
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion rotation #旋转向量（四元数）
        float64 x
        float64 y
        flaot64 z
        float64 w        */
```





## pluginlib

### 1、概念

pluginlib是一个c++库， 用来从一个ROS功能包中加载和卸载插件(plugin)。插件是指从运行时库中动态加载的类。通过使用Pluginlib，不必将某个应用程序显式地链接到包含某个类的库，Pluginlib可以随时打开包含类的库，而不需要应用程序事先知道包含类定义的库或者头文件。

其实对于ROS-1初学者来说，写得最多的是什么？没错，是节点！因为ROSWIKI上的入门教程都是教你如何写节点程序的，比如一个发布者，或是一个服务器。但是随着你编写的程序越来越复杂，实现的功能模块越来越齐全，你就会发现写节点太过于分散了，每次需要启动一大堆的节点，而这些节点很可能在场景中的地位是平等且并行的，这时候你就会想到，有没有可能用一种方式把这些节点像浏览器书签一样管理，想用哪个就把哪个调出来，动态的进行实例化和加载使用。幸运的是，ROS-1开发者也预料到了这种使用场景，所以他们提供了pluginlib这个c++库，其底层就是帮我们实现了一套工厂模式！！

在导航中，涉及到路径规划模块，路径规划算法有多种，也可以自实现，导航应用时，可能需要测试不同算法的优劣以选择更合适的实现，这种场景下，ROS中就是通过插件的方式来实现不同算法的灵活切换的。

### 2、写法

* 创建插件基类，定义统一接口（如果为现有接口编写插件，则跳过该步）
* 编写插件类，继承插件基类，实现统一接口
* 导出插件，并编译为动态库
* 将插件加入ROS系统，使其可识别和管理

### 3、注意事项







## nodelet

ROS通信是基于Node(节点)的，Node使用方便、易于扩展，可以满足ROS中大多数应用场景，但是也存在一些局限性，由于一个Node启动之后独占一根进程，不同Node之间数据交互其实是不同进程之间的数据交互，当传输类似于图片、点云的大容量数据时，会出现延时与阻塞的情况，比如：

现在需要编写一个相机驱动，在该驱动中有两个节点实现:其中节点A负责发布原始图像数据，节点B订阅原始图像数据并在图像上标注人脸。如果节点A与节点B仍按照之前实现，两个节点分别对应不同的进程，在两个进程之间传递容量可观图像数据，可能就会出现延时的情况，那么该如何优化呢？ROS中给出的解决方案是:Nodelet，通过Nodelet可以将多个节点集成进一个进程。

nodelet本质也是插件，实现流程与插件实现流程类似，并且更为简单，不需要自定义接口，也不需要使用类加载器加载插件类。



## XML-RPC

### 1、概念

XML-RPC 是一种远程过程调用协议，XML 指可扩展标记语言，RPC 指远程过程调用。（其实现在 JSON-RPC 更为较常用）

它允许在不同的计算机系统之间进行通信，就像在同一个系统中进行函数调用一样。使用 XML 来编码参数和返回值，使用 HTTP 作为传输协议。定义了如何将一个RPC请求编码为XML，以及如何将一个RPC响应解码为XML。它支持包括整数、浮点数、布尔值、字符串、数组和字典在内的基本数据类型，以及数组和字典的嵌套。

一个rpc系统，必然包括2个部分：

* **rpc client**：用来向 rpc server 调用方法，并接收方法的返回数据。工作原理：rpcclient 根据 URL 找到rpcserver -> 构造命令包，调用rpcserver上的某个服务的某个方法 -> 接收到rpcserver的返回,解析响应包，拿出调用的返回结果。
* **rpc server**：用于响应 rpc client 的请求，执行方法，并回送方法执行结果。工作原理：启动动一个webserver(在使用内置的webserver的情况下) -> 注册每个能提供的服务，每个服务对应一个Handler类 ->进入服务监听状态。



### 2、语法

* 标量数据类型(scalar )

  * 参数值<value>可以是标量，用类型标签将值包括起来。如果没指定类型，则认为是string类型。
  * <i4>或者 <int>表示 4字节带符号整数值  
  * <boolean>表示 0 (false) or 1 (true) 
  * <string>表示   字符串  
  * <double>表示   双精度带符号浮点值  
  * <dateTime.iso8601>表示 日期/时间
  * <base64>表示 base64编码的二进制数据

* 结构数据类型(<struct>)

  参数值也可以是<struct>类型一个<struct>可含有几个<member>项，每个< member>含有一个<name>项和一个<value>项。<member>的< value>值可以为任何类型，可为标量类型、<array>甚至<struct>(即可以递归).

* 数组数据类型(<array>)

  参数值也可以是<array>类型一个<array>含有单一的 <data> 元素，<data>元素可以含有任意数量的<value>, 这里的<value>没有name.每个<value>的数据类型可各不相同。<data>的<value>值可以为任何类型，可为标量类型、 <struct>甚至<array>(即可以递归).



### 3、XmlRpcValue

XMLRPC++ 代码中与此对应的是类 XmlRpc::XmlRpcValue，XMLRPC 执行远程调用时，其输入参数与执行结果均封装在 XmlRpcValue 对象中。客户端将需执行的方法，以及方法参数（XmlRpcValue）以XML格式（通过HTTP协议）传输到服务器端，服务器解析 XML，获得以 XmlRpcValue 封装的参数，在服务器端调用方法，获得以 XmlRpcValue 封装的执行结果，将其以XML格式传输至客户端，客户端解析，获得执行结果（XmlRpcValue）。：

* **客户端**：参数 －> XmlRpcValue params－> XML －> HTTP协议传输 －> XML －>XmlRpcValue params－>服务器端参数
* **服务器端**：执行结果 －> XmlRpcValue result－> XML －> HTTP协议传输 －> XML －>XmlRpcValue result－>客户端执行结果

```cpp
enum Type {
       TypeInvalid,// 非法类型
       TypeBoolean,// 0 (false) or 1 (true)
       TypeInt,// 4字节带符号整数值
       TypeDouble,// 双精度带符号浮点值   
       TypeString,// 字符串   
       TypeDateTime,// 日期/时间
       TypeBase64,// base64编码的二进制数据
       TypeArray, // 数组数据类型
       TypeStruct // 结构数据类型
     };
     typedef std::vector<char> BinaryData; // 二进制数据
     typedef std::vector<XmlRpcValue> ValueArray; // 数组
     typedef std::map<std::string, XmlRpcValue> ValueStruct;// 结构
```

一个 XmlRpcValue 对象只能有一个值，通过一个联合来定义，如下：

```cpp
union {
   bool   asBool
   int   asInt
   double   asDouble
   tm *   asTime
   std::string *   asString
   BinaryData *   asBinary
   ValueArray *   asArray
   ValueStruct *   asStruct
} _value;
```



类 XmlRpcValue 提供的方法包括如下几类：

* 从 xml 中获得数据，如 boolFromXml，intFromXml，doubleFromXml 等

* 将数据转换成 xml 格式，如 boolToXml，intToXml，doubleToXml 等（继承自 XmlRpcSource）

* 获取数据类型 `getType()`：

  ```cpp
  if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
  ```

* 类型检查，如assertArray，assertStruct等

* 判断是否有成员 `hasMember()`：

  ```cpp
  if(behavior_list[j].hasMember("name"))
  ```

* 操作符重载，如赋值＝，＝＝，！＝等，若为数组，则可使用 [] 提取指定元素

  ```cpp
  std::string name_j = behavior_list[j]["name"];
  ```

* 其他

### 4、其它重要的类

* **XmlRpcServerMethod**：服务器支持的调用方法的基类
* **RpcSource**：RPC处理的端点

* **XmlRpcClient**：RPC客户端 
* **XmlRpcServer**：RPC服务器
* **XmlRpcServerConnection**：RPC服务器连接类
* **XmlRpcDispatch**：维护一个当前被监视的对象列表，集中处理





## dynamic_reconfigure：动态参数

dynamic_reconfigure配置是ROS中为了方便用户对程序中的参数进行实时调整而推出的工具，配置好自己的dynamic_reconfigure文件后，可以很方便的使用ROS提供的rqt_reconfigure工具对程序的参数进行合理调整，以获得最优的性能。

例如：move_base 中就针对 costmap、planner 等设置了很多动态调整的参数，可以方便用户在使用过程中调整得到合适的参数。

又例如：如果我们开发了一个PID控制器程序，如果这时能通过 rqt_reconfigure 工具，对PID参数进行合理的调整，然后直接将获得的参数写进程序，就能使工作更加高效。



引入头文件：

* dynamic_reconfigure 服务器的头文件，用于创建服务器
* 自己写的 .cfg 文件生成的头文件，用于使用我们定义的参数

然后，我们可以创建两个变量，第一个是动态配置参数服务器，第二个是回调函数，也就是我们使用 rqt_reconfigure 调参时进入的函数，需要我们自己编写。




















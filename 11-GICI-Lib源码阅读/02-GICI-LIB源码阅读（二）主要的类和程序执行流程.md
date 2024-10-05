> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、cpp 语法知识补充

### 1、std::function()、std::bind()

`std::function` 和 `std::bind` 是 cpp 中用于函数包装和绑定的工具，在 GICI中用于绑定数据回调函数。

- `std::function` 是一个可调用对象，可以存储任何可调用对象，例如函数指针、成员函数指针或 lambda 表达式。您可以使用 `std::function` 来创建一个函数对象，以便稍后调用。例如：

  ```cpp
  #include <iostream>
  #include <functional>
  
  int add(int a, int b) {
      return a + b;
  }
  
  int main() {
      std::function<int(int, int)> func = add;
      int result = func(2, 3);
      std::cout << result << std::endl; // Output: 5
      return 0;
  }
  ```

  在上面的例子中，我们使用 `std::function` 来存储 `add` 函数，并在 `main` 函数中调用它。

- `std::bind` 用于将函数和其参数绑定到特定的对象。它返回一个可调用对象，可以将其传递给 `std::function` 或直接调用。例如：

  ```cpp
  #include <iostream>
  #include <functional>
  
  class MyClass {
  public:
      void myFunction(int a) {
          std::cout << "MyClass::myFunction called with " << a << std::endl;
      }
  };
  
  int main() {
      MyClass obj;
      std::function<void(int)> func = std::bind(&MyClass::myFunction, &obj, 42);
      func(); // Output: MyClass::myFunction called with 42
      return 0;
  }
  ```

  在上面的例子中，我们使用 `std::bind` 将 `MyClass::myFunction` 函数绑定到 `obj` 对象上，并将参数 `42` 传递给它。然后，我们将绑定后的函数存储到 `std::function` 中，并在 `main` 函数中调用它。

### 2、STL 容器

- **vector**：动态数组，大小可动态调整，支持快速随机访问和插入/删除操作。
- **list**：双向链表，支持在任意位置插入/删除元素，但不支持随机访问。
- **deque**：双端队列，支持在两端插入/删除元素，也支持快速随机访问。
- **set**：红黑树实现的有序集合，支持快速查找和插入/删除操作。
- **multiset**：类似于set，但允许重复元素。
- **map**：红黑树实现的有序映射，支持快速查找和插入/删除操作。
- **multimap**：类似于map，但允许重复的键。
- **unordered_set**：哈希表实现的无序集合，支持快速查找和插入/删除操作。
- **unordered_multiset**：类似于unordered_set，但允许重复元素。
- **unordered_map**：哈希表实现的无序映射，支持快速查找和插入/删除操作。
- **unordered_multimap**：类似于unordered_map，但允许重复的键。

### 3、智能指针

GICI 中大量的使用了智能指针来管理对象。智能指针用来帮助程序员管理动态分配的内存，可以确保在程序退出使用一块内存之前，该内存会被正确释放，从而避免内存泄漏。 

- `std::unique_ptr`：独占式智能指针，它拥有对一个对象的独占所有权。当不再需要该指针时，它将自动释放其所拥有的对象。GICI 创建线程的时候都用 `std::unique_ptr`。
- `std::shared_ptr`：共享式智能指针，用于多个指针共享同一个对象。当最后一个持有`shared_ptr`的指针被销毁时，其所指向的对象才会被销毁。GICI 中使用的最多，对象基本都用此来管理，还用 `using` 给智能指针起别名 `xxxPtr`。
- `std::weak_ptr`：弱引用指针，用于避免循环引用。它不会增加对象的引用计数，也不会拥有对象的所有权。它只能用于观察对象，不能直接访问。

## 二、主要的类

> 想彻底看懂面向对象的程序，先要看懂它的类。
>
> 原始 XMind 思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

### 1、NodeOptionHandle 类：管理配置文件信息

![1690183598997](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690183598997.png)

### 2、StreamerBase 类：数据流读写

StreamerBase 是 各种 Streamer 对象的基类，用于传输数据流，支持 I/O 端口、串口、ROS topics、TCP/IP、NTRIP、V4L2、文件。每个数据流都需要一个 streamer  对象，与配置文件中定义的 streamer 对应。

![1690184966827](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690185815380.png)

### 3、FormatorBase 类：数据解码编码

FormatorBase 是 各种 Formator 对象的基类，用于解析从数据流里读取的数据，支持 RTCM2、RTCM3、 Ublox raw、Septentrio raw、Tersus raw、NMEA、DCB file、ATX file、V4L2 image pack、GICI image pack、GICI IMU pack。每个数据格式都需要一个 Formator 对象，与配置文件中定义的 Formator 对应。

![1690187409101](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690187409101.png)

### 4、EstimatorBase 类：定位解算

EstimatorBase 是 各种 Estimator 对象的基类，用于进行定位解算。除了配置文件中定义的解算模式对应的 Estimator，每种模式都会创建 SPP 估计器用于获取初始坐标；GNSS/INS 初始化要创建初始化估计器；松组合要创建 RTK 估计器，用以计算 GNSS-RTK 与 IMU/Camera 组合。

![image-20230927155151067](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230927155151067.png)

### 5、DataClusters 类

![1690185521245](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690185521245.png)

### 6、EstimatorDataCluster 类

所有用于解算的传感器数据都存成 EstimatorDataCluster，在 MultiSensorEstimating 中有字段：

```cpp
// Data buffers 数据缓冲区
std::deque<EstimatorDataCluster> measurements_;  // non propagate measurements
int last_backend_pending_num_ = 0;
// the frontend measurements should be processd by frontend, and the output of frontend will 
// be inserted into measurements_.
std::deque<EstimatorDataCluster> image_frontend_measurements_; 
int last_image_pending_num_ = 0;
// buffer to temporarily store measurements in case the input stream blocking
std::deque<EstimatorDataCluster> measurement_addin_buffer_;
// buffer to align timestamps of different sensor streams
std::deque<EstimatorDataCluster> measurement_align_buffer_;
double latest_imu_timestamp_;
```

![1690203925616](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690203925616.png)

### 5、NodeHandle 类：Streaming、Estimating 线程管理与交互

![1690291450714](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690291450714.png)

### 6、Streaming 类：Streaming 线程管理

![1690292045851](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690292045851.png)

### 7、DataIntegrationBase 类：数据融合

在 NodeHandle 类对象内有字段：

```cpp
std::vector<std::vector<std::shared_ptr<DataIntegrationBase>>> data_integrations_;
```

数据集成处理根据其角色打包数据并将其发送给估算器，outter 向量与 estimatings_ 对齐，后者描述数据目的地，内部向量与相应估计器的输入节点对齐。

![1690292848614](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690292848614.png)

### 8、EstimatingBase 类

用于管理 Estimating 线程，是 MultiSensorEstimating 类的基类。

![1690293594836](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690293594836.png)

### 9、MultiSensorEstimating 类：Estimating 线程管理

GICI 的核心线程。

![1690294818955](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690294818955.png)

### 10、SpinControl 类：线程 spin 控制

GICI 内的线程都是处于 spin 状态，就是每隔一定时间，判断一次有没有数据、需不需要执行对应的函数。SpinControl 用于控制每次休眠的时间、结束线程等。

![1690203866545](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690203866545.png)

## 三、程序执行流程

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230807135432767.png)

### 1、线程设计

程序运行主要基于两个线程：stream 线程、estimator 线程，都由 NodeHandle 管理，并在构造函数中创建：

- **stream 线程**：处理数据输入输出、解码编码。 在 `Streaming::start()` 函数中创建，执行 `Streaming::run` 函数。
- **estimator 线程**：处理由 stream 线程获取的原始数据，并将解算结果传回 stream 线程。在 `EstimatingBase::start()` 函数中创建，执行 `EstimatingBase::run` 函数。一个 estimator 线程又包含三个线程：frontend、backend、，都由 MultiSensorEstimating 管理，`在 MultiSensorEstimating::process` 函数中创建。
  - **frontend 线程**：进行预处理，最耗时，尤其是针对视觉信息的特征提取和跟踪。执行 `runImageFrontend` 函数。
  - **backend 线程**：进行基于图优化参数估计，结果可用于 frontend 线程，提高特征提取和跟踪的质量。执行 `runBackend` 函数。
  - **export 线程**：将 backend 线程估计得到结果进行集成，转化成缩需要的采样率，以进行输出、存储。执行 `runMeasurementAddin` 函数。

### 2、mian 函数

- 程序从 gici_main.cpp 中的 `main()` 函数开始执行。
- `main()` 函数首先根据参数数目判断是否传入配置文件路径。
- 调用 `YAML::LoadFile()` 加载配置文件到 `yaml_node`。
- 根据配置文件中 `logging` 部分设置，初始化 glog 日志。
- 调用 `initializeSignalHandles()` 注册 SIGPIPE 和 SIGSEGV 的信号处理函数。处理方式就是向日志文件写错误信息，并退出程序。
  - `SIGPIPE` 信号：向已关闭的 socket 写数据。
  - `SIGSEGV `信号：向不存在的内存写数据。
  - `sigemptyset()`：用于初始化一个信号集，该信号集不包含任何信号。
  - `sigaction()`：设置信号处理回调函数。
- 用 `make_sharedmake_shared<NodeOptionHandle>` 通过 `yaml_node` 构造 `NodeOptionHandle` 对象，并用 `node_option_handle` 智能指针指向它。`NodeOptionHandle` 的构造函数很长，会在下面具体分析，在此构造函数中导入配置文件各节点的内容，并且对内容进行检查，检查的结果体现在 `valid` 字段中。
- 用 `make_unique<NodeHandle>` 通过 `node_option_handle` 构造 `NodeHandle` 对象，并用 `node_handle` 智能指针指向它。`NodeHandle` 的构造函数也很长，会在下面具体分析，在此函数的最后：
  - 通过`streamings_[i]->start();` 开启了 `stream` 线程
  - 通过 `estimatings_[i]->start();` 开启了 `estimator` 线程
- 输出初始化的信息：streamer、formater、estimator 大小。
- 调用 `SpinControl::run() ` 以 spin 线程方程启动所有线程，
- 创建 SpinControl 对象，进入 spin 状态。

```cpp
// Usage: <path>/gici_main <path-to-option>. 
int main(int argc, char** argv)
{
  // Get option file
  if (argc != 2) { 
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-option>" << std::endl;
    return -1;
  }
  std::string config_file_path = argv[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);  // 加载配置文件
  } catch (YAML::BadFile &e) {
    std::cerr << "Unable to load option file!" << std::endl;
    return -1;
  }

  // 根据配置文件中 logging 部分设置，初始化 glog 日志
  // Initialize glog for logging
  bool enable_logging = false;
  if (yaml_node["logging"].IsDefined() && 
      option_tools::safeGet(yaml_node["logging"], "enable", &enable_logging) && 
      enable_logging == true) {
    YAML::Node logging_node = yaml_node["logging"];   // 取 logging 节点到 logging_node
    google::InitGoogleLogging("gici");  //初始化glog日志
    int min_log_level = 0;
    if (option_tools::safeGet(
        logging_node, "min_log_level", &min_log_level)) {
      FLAGS_minloglevel = min_log_level;
    }
    option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
    option_tools::safeGet(logging_node, "file_directory", &FLAGS_log_dir);
    if (FLAGS_logtostderr) FLAGS_stderrthreshold = min_log_level;
    else FLAGS_stderrthreshold = 5;
  }

  // 注册 SIGPIPE 和 SIGSEGV 的信号处理函数。
  // 处理方式就是向日志文件写错误信息，并退出程序。
  // Initialize signal handles to catch faults
  initializeSignalHandles();
  
  // Organize nodes node_option_handle 管理各个node之间的关系
  NodeOptionHandlePtr node_option_handle = 
    std::make_shared<NodeOptionHandle>(yaml_node);  // 这里会加载配置文件
  if (!node_option_handle->valid) {
    std::cerr << "Invalid configurations!" << std::endl;
    return -1;
  }

  // Initialize nodes
  std::unique_ptr<NodeHandle> node_handle = 
    std::make_unique<NodeHandle>(node_option_handle); //这里会创建streamer和estimate的线程 

  // Show information 
  const std::vector<size_t> sizes = {
    node_option_handle->streamers.size(),
    node_option_handle->formators.size(), 
    node_option_handle->estimators.size()};
  std::cout << "Initialized " 
    << sizes[0] << " streamer" << (sizes[0] > 1 ? "s" : "") << ", "
    << sizes[1] << " formater" << (sizes[1] > 1 ? "s" : "") << ", and "
    << sizes[2] << " estimator" << (sizes[2] > 1 ? "s" : "") << ". "
    << "Running..." << std::endl;

  // gici-open在主函数里没有给出任何终止程序的代码，
  // 所以哪怕是运行结束了，也是在一直等待，需要自己手动退出
  // Start running all threads
  SpinControl::run(); // 自旋锁

  // Loop
  SpinControl spin(1e-1);
  while (SpinControl::ok()) {
    spin.sleep();
  }

  return 0;
}
```

### 3、NodeOptionHandle 构造函数

- 传入 `LoadFile()` 加载配置文件得到的 `yaml_node`。
- 根据配置文件中的 `streamer`、`formator`、`estimator` 构造对应的对象 StreamerNodeBase、FormatorNodeBase、EstimatorNodeBase。其中会调用构造函数：
  - 先调用父类 NodeBase 的构造函数：传入 yaml_node，获取各配置项到对应字段，并将 yaml_node 赋值给 this_node。
  - 子类的构造函数中会对类型进行验证，然后 FormatorNodeBase 会获取 io 配置项，EstimatorNodeBase 会获取 input_tag_roles 配置项。
- 分别加到 streamers、formators、estimators 字段。
- 全都转换成基类 NodeBase 对象加到 nodes 字段。
- 用 nodes 中存的对象和对象的 tag 构建 map， 存到 tag_to_node 字段。
- 将 replay 节点存到 replay_options 字段。
- 最后整理节点之间的连接关系、判断配置是否合理，判断的结果存到 valid 字段。

```cpp
NodeOptionHandle::NodeOptionHandle(const YAML::Node& yaml_node) :
  valid(true)
{
  // Load streamers and formators
  if (yaml_node["stream"].IsDefined())  // 判断 stream 节点是否存在
  {
    // 取 stream 节点到 stream_node
    const YAML::Node& stream_node = yaml_node["stream"];   

    // Load streamers
    if (stream_node["streamers"].IsDefined()) { // 判断 streamers 节点是否存在
      const YAML::Node& streamer_nodes = stream_node["streamers"];  // 取 streamers 节点到 streamer_nodes
      // 遍历 streamers 节点的 streamer
      for (size_t i = 0; i < streamer_nodes.size(); i++) {
        const YAML::Node& streamer_node = streamer_nodes[i]["streamer"];  // 取 streamer 节点 到 streamer_node
        StreamerNodeBasePtr streamer = 
          std::make_shared<StreamerNodeBase>(streamer_node);  // 其中会调用 StreamerNodeBase 的构造函数，加载 streamer 的配置项
        streamers.push_back(streamer); // 将此次循环读取的的一个 streamer 存入 streamers
        nodes.push_back(std::static_pointer_cast<NodeBase>(streamer));
        tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
      }
    }

    // Load formators
    if (stream_node["formators"].IsDefined()) { // 判断 formators 节点是否存在
      const YAML::Node& formator_nodes = stream_node["formators"];  // 取 formators 节点到 formator_nodes
      // 遍历 formator_nodes 节点的 formator
      for (size_t i = 0; i < formator_nodes.size(); i++) { 
        const YAML::Node& formator_node = formator_nodes[i]["formator"]; // 取 formator 节点 到 formator_node
        FormatorNodeBasePtr formator = 
          std::make_shared<FormatorNodeBase>(formator_node);  // 其中会调用 FormatorNodeBase 的构造函数，加载 formator 的配置项
        formators.push_back(formator);  // 将此次循环读取的的一个 streamer 存入 streamers
        nodes.push_back(std::static_pointer_cast<NodeBase>(formator));
        tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
      }
    }

    // Relay options
    if (stream_node["replay"].IsDefined()) {  // 判断 replay 节点是否存在
      replay_options = stream_node["replay"]; // 取 replay 到 replay_options 字段
    }
  }

  // Load estimators
  if (yaml_node["estimate"].IsDefined()) { 
    const YAML::Node& estimator_nodes = yaml_node["estimate"];
    for (size_t i = 0; i < estimator_nodes.size(); i++) {
      const YAML::Node& estimator_node = estimator_nodes[i]["estimator"];
      EstimatorNodeBasePtr estimator = 
        std::make_shared<EstimatorNodeBase>(estimator_node);
      estimators.push_back(estimator);
      nodes.push_back(std::static_pointer_cast<NodeBase>(estimator));
      tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
    }
  }

  // Check all nodes 检查每一个 streamer、formator、estimator 的 valid 字段是否为 true
  if (!checkAllNodeOptions()) { valid = false; return; }

  // 整理节点之间的连接关系。代码中通过遍历节点智能指针来处理每个节点的输入和输出标签。
  // 首先，对于每个节点，如果其输入标签的数量大于0，则遍历所有节点，并检查是否存在与输入标签匹配的节点标签。
  // 如果存在匹配的节点标签且该节点的输出标签中不存在当前节点的标签，则将当前节点的标签添加到匹配节点的输出标签中。
  // 接下来，对于每个节点的输出标签，同样遍历所有节点，并检查是否存在与输出标签匹配的节点标签。
  // 如果存在匹配的节点标签且该节点的输入标签中不存在当前节点的标签，则将当前节点的标签添加到匹配节点的输入标签中。
  // 这样一轮遍历之后，就完成了节点之间连接关系的整理。
  // Organize connections
  for (size_t i = 0; i < nodes.size(); i++) { // 通过智能指针遍历所有 Node
    if (nodes[i]->input_tags.size() > 0)            
    for (auto& input_tag : nodes[i]->input_tags) { 
      for (size_t j = 0; j < nodes.size(); j++) {
        if (nodes[j]->tag != input_tag) continue;
        if (!tagExists(nodes[j]->output_tags, nodes[i]->tag)) {
          nodes[j]->output_tags.push_back(nodes[i]->tag);
        }
      }
    }
    for (auto& output_tag : nodes[i]->output_tags) {
      for (size_t j = 0; j < nodes.size(); j++) {
        if (nodes[j]->tag != output_tag) continue;
        if (!tagExists(nodes[j]->input_tags, nodes[i]->tag)) {
          nodes[j]->input_tags.push_back(nodes[i]->tag);
        }
      }
    }
  }


  // Check if connections valid
  for (size_t i = 0; i < nodes.size(); i++) { // 通过智能指针遍历所有 Node，判断是否合规
    if (nodes[i]->input_tags.size() == 0 && nodes[i]->output_tags.size() == 0) {
      LOG(ERROR) << nodes[i]->tag << ": " 
        << "At least one input tag or output tag should be specified!";
      valid = false; return;
    }

    int n_out_fmts = 0, n_out_strs = 0, n_out_ests = 0;
    int n_in_fmts = 0, n_in_strs = 0, n_in_ests = 0;
    for (auto& output_tag : nodes[i]->output_tags) {
      if (output_tag.substr(0, 4) == "fmt_") n_out_fmts++;
      if (output_tag.substr(0, 4) == "str_") n_out_strs++;
      if (output_tag.substr(0, 4) == "est_") n_out_ests++;
      if (tag_to_node.find(output_tag) == tag_to_node.end()) {
        LOG(ERROR) << nodes[i]->tag << ": " 
          << "Cannot find the node with tag " << output_tag << "!";
        valid = false; return;
      }
    }
    for (auto& input_tag : nodes[i]->input_tags) {
      if (input_tag.substr(0, 4) == "fmt_") n_in_fmts++;
      if (input_tag.substr(0, 4) == "str_") n_in_strs++;
      if (input_tag.substr(0, 4) == "est_") n_in_ests++;
      if (tag_to_node.find(input_tag) == tag_to_node.end()) {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Cannot find the node with tag " << input_tag << "!";
        valid = false; return;
      }
    }
    if (nodes[i]->node_type == NodeType::Streamer) {
      if (n_out_ests > 0 && nodes[i]->type != "ros") {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Only ROS streamer is allowed to output to estimator!";
        valid = false; return;
      }
      if (n_in_strs > 1) {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "A streamer can connect to only one streamer input!";
        valid = false; return;
      }
      if (n_in_ests > 0 && nodes[i]->type != "ros") {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Only ROS streamer is allowed to input from estimator!";
        valid = false; return;
      }
    }
    if (nodes[i]->node_type == NodeType::Formator) {
      FormatorNodeBasePtr formator = 
        std::static_pointer_cast<FormatorNodeBase>(nodes[i]);
      if (formator->io == "input") {
        /* Allowed for ROS streamer.
        if (n_out_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to streamers is not allowed by formator in input mode!";
          valid = false; return;
        } */
        if (n_in_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from formators is not allowed by formator in input mode!";
          valid = false; return;
        }
        if (n_in_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in input mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_in_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from estimators is not allowed by formator in input mode!";
          valid = false; return;
        }
      }
      else if (formator->io == "log") {
        if (n_out_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to formators is not allowed by formator in log mode!";
          valid = false; return;
        }
        if (n_out_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_out_strs == 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode should connect to one stream!";
          valid = false; return;
        }
        if (n_out_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to estimators is not allowed by formator in log mode!";
          valid = false; return;
        }
        if (n_in_fmts > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one formator!";
          valid = false; return;
        }
        if (n_in_fmts == 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode should connect to one formator!";
          valid = false; return;
        }
        if (n_in_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from streamers is not allowed by formator in input mode!";
          valid = false; return;
        }
        if (n_in_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from estimators is not allowed by formator in input mode!";
          valid = false; return;
        }
      }
      else if (formator->io == "output") {
        if (n_out_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to formators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_out_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in output mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_out_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to estimators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from formators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from streamers is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_ests > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one estimator!";
          valid = false; return;
        }
      }
    }
    if (nodes[i]->node_type == NodeType::Estimator) {
      // already checked by above
    }
  }

  // Check estimator to estimator connection


  // We need at least one streamer
  if (streamers.size() == 0) {
    LOG(ERROR) << "At least one streamer should be specified!";
    valid = false; return;
  }
}
```

### 4、NodeHandle 构造函数

- 传入 NodeOptionHandle 智能指针。
- 遍历 NodeOptionHandle 中的 streamers，如果不是 ROS 类型，创建 Streaming 对象，将智能指针加到 streamings_ 。
- 遍历 NodeOptionHandle 中的 estimators，创建 MultiSensorEstimating 对象，将智能指针加到 estimatings_ 。
- 调用 bindLogWithInput、bindStreamerToFormatorToEstimator、bindEstimatorToFormatorToStreamer、bindEstimatorToEstimator 绑定回调函数。
- 根据三个配置项：enable、speed、start_offset，设置 replay，调用 enableReplay 
- 循环调用 `start()` 成员函数，开启所有 streaming、estimating 线程。

```cpp
NodeHandle::NodeHandle(const NodeOptionHandlePtr& nodes)
{
  // 初始化 stream 线程
  // Initialize streaming threads (formators are initialized together)
  for (size_t i = 0; i < nodes->streamers.size(); i++) { // 遍历配置文件中的 streamer
    // check if ROS streamer 
    // 获取 streamer 的种类，并判断是不是 ROS，如果是 ROS 则跳出本次循环
    std::string type_str = nodes->streamers[i]->type;
    StreamerType type;
    option_tools::convert(type_str, type);
    if (type == StreamerType::Ros) continue;

    // 创建 streamer 线程，并将其智能指针加到 streamings_ vector
    // 会调用 Streaming 的构造函数
    auto streaming = std::make_shared<Streaming>(nodes, i);
    if (!streaming->valid()) continue;
    streamings_.push_back(streaming);
  }

  // 初始化 estimator 线程
  // Initialize estimator threads
  for (size_t i = 0; i < nodes->estimators.size(); i++) { // 遍历配置文件中的 estimator
    // 获取 streamer 的种类
    std::string type_str = nodes->estimators[i]->type;
    EstimatorType type;
    option_tools::convert(type_str, type);

    // 创建 estimator 线程，并将其智能指针加到 estimatings_ vector
    // 会调用 MultiSensorEstimating 的构造函数
    if (type != EstimatorType::None) {
      auto estimating = std::make_shared<MultiSensorEstimating>(nodes, i);
      estimatings_.push_back(estimating);
    }
  }

  // Bind streamer->streamer, streamer->formator->formator->streamer pipelines
  Streaming::bindLogWithInput();

  // Bind streamer->formator->estimator pipelines
  bindStreamerToFormatorToEstimator(nodes);

  // Bind estimator->formator->streamer pipelines
  bindEstimatorToFormatorToStreamer(nodes);

  // Bind estimator->estimator pipelines
  bindEstimatorToEstimator(nodes);

  // 设置 replay，根据三个配置项：enable、speed、start_offset
  // Get replay option and enable replay
  bool enable_replay = false;
  StreamerReplayOptions replay_options;
  if (!nodes->replay_options.IsDefined() || 
      !option_tools::safeGet(nodes->replay_options, "enable", &enable_replay)) {
    LOG(INFO) << "Unable to load replay options. Disable replay!";
  }
  if (enable_replay) {
    if (!option_tools::safeGet(nodes->replay_options, "speed", &replay_options.speed)) {
      LOG(INFO) << "Unable to load replay speed! Using default instead";
      replay_options.speed = 1.0;
    }
    if (!option_tools::safeGet(nodes->replay_options, "start_offset", 
        &replay_options.start_offset)) {
      LOG(INFO) << "Unable to load replay start offset! Using default instead";
      replay_options.start_offset = 0.0;
    }
    Streaming::enableReplay(replay_options);
  }

  // Start streamings  开启 streamings 线程
  for (size_t i = 0; i < streamings_.size(); i++) {
    streamings_[i]->start();
  }

  // Start estimators  开启 estimators 线程
  for (size_t i = 0; i < estimatings_.size(); i++) {
    estimatings_[i]->start();
  }
}
```

### 5、Streaming 线程

#### 1. Streaming  构造函数

- 根据传入的 i_streamer 索引从nodes 中取出对应的 streamer 配置选项到 node、streamer_node。
- 调用 makeStreamer() 先判断配置项是否存在，再根据配置选项创建对应的 streamer 对象，（Serial、File 等）加到 streamer_ 字段。
- 如果有 formator
  - 调用 makeFormator() 创建对应的 FormatorBase 对象（RTCM2、IMUPack 等），与 tag、IO 种类一起构建 formator_ctrl 结构体，加到 formators_  字段。
  - 创建空对象加到 data_clusters_。
  - 根据 IO 种类（Input、Log、Output）将对应的标识字段（`has_input_`、`has_logging_`、`has_output_`）设为 true
- 遍历 node，如果其 output_tags 是 str_xxx，加到 output_streamer_tags，output_streamer_tags 有元素，any_option_got、has_logging_ 设为 true
- 对 input_tags 也做上述操作。
- 根据配置选项 buffer_length 设置 max_buf_size_ 
- 如果 input、logging、output 都没有，就释放空间并 LOG(FATAL) 报错退出程序 
- 获取配置选项中 start_message 到 message
- 根据数据流读写模式（R\W\RW）调用成员函数 open 开启数据流 
- 把 message 转为 uint8_t，调用成员函数 write 发 start_message 数据 
- 如果前面执行都顺利，valid_ 字段设为 true。

```cpp
Streaming::Streaming(const NodeOptionHandlePtr& nodes, size_t i_streamer) : 
  valid_(false), opened_(false)
{
  // 根据 streamer 选项创建对应的 streamer 对象，加到 streamer_ 字段
  // Get streamer option  获取 streamer 选项
  const auto& node = nodes->streamers[i_streamer];
  tag_ = node->tag;
  YAML::Node streamer_node = node->this_node;

  // Initialize streamer   根据 streamer 类型创建对应的 streamer 对象
  streamer_ = makeStreamer(streamer_node);
  if (streamer_ == nullptr) {
    LOG(ERROR) << tag_ << ": Unable to make streamer!";
    return;
  }

  bool any_option_got = false;

  // 根据 formater 选项创建 formator_ctrl 对象，加到 formators_ 字段里
  // Get formator option 获取 formator 选项
  std::vector<std::string> formator_tags;
  const std::vector<std::string>& tags = node->tags();
  for (const auto& tag : tags) {
    if (tag.substr(0, 4) == "fmt_") formator_tags.push_back(tag);
  }
  if (formator_tags.size() > 0) {
    any_option_got = true;

    // Initialize formators
    for (auto& tag : formator_tags) {
      NodeOptionHandle::FormatorNodeBasePtr formator_node = 
        std::static_pointer_cast<NodeOptionHandle::FormatorNodeBase>
        (nodes->tag_to_node.at(tag));
      std::string type_str = formator_node->io;
      StreamIOType type;
      option_tools::convert(type_str, type);  // 获取 IO 种类（Input、Output、Log）

      
      std::shared_ptr<FormatorBase> formator = makeFormator(formator_node->this_node);
      // 构建 formator_ctrl 结构体，加到 formators_  字段
      FormatorCtrl formator_ctrl;
      formator_ctrl.formator = formator;
      formator_ctrl.tag = formator_node->tag;
      formator_ctrl.type = type;
      // If log or output stream, find and handle corresponding input stream as well
      if (type == StreamIOType::Log || type == StreamIOType::Output) {
        if (formator_node->input_tags.size() > 0) {
          CHECK(formator_node->input_tags.size() == 1);
          formator_ctrl.input_tag = formator_node->input_tags.back();
        }
      }
      formators_.push_back(formator_ctrl);

      // 创建空对象加到 data_clusters_
      data_clusters_.push_back(std::vector<std::shared_ptr<DataCluster>>());

      // 根据 IO 种类（Input、Log、Output）将对应的标识字段（has_input_、has_logging_、has_output_）设为 true
      if (type == StreamIOType::Input) has_input_ = true;
      if (type == StreamIOType::Log) has_logging_ = true;
      if (type == StreamIOType::Output) has_output_ = true;
    }
  }

  // Get output streamer for direct logging
  std::vector<std::string> output_streamer_tags;
  for (const auto& tag : node->output_tags) { // 遍历 node，如果其 output_tags 是 str_xxx，加到 output_streamer_tags
    if (tag.substr(0, 4) == "str_") output_streamer_tags.push_back(tag);
  }
  if (output_streamer_tags.size() > 0) {  // output_streamer_tags 有元素，any_option_got、has_input_ 设为 true
    any_option_got = true;
    has_input_ = true;  // ？？？？？？？？？？？？
  }

  // Get input streamer for direct logging
  std::vector<std::string> input_streamer_tags;
  for (const auto& tag : node->input_tags) {
    if (tag.substr(0, 4) == "str_") input_streamer_tags.push_back(tag);
  }
  CHECK(input_streamer_tags.size() <= 1);
  if (input_streamer_tags.size() > 0) {
    input_tag_ = input_streamer_tags.back();
    any_option_got = true;
    has_logging_ = true;
  }

  if (!any_option_got) {
    LOG(ERROR) << tag_ << ": Unable to load either output_tags nor input_tags!";
    return;
  }

  // Initialize buffer 根据配置选项 buffer_length 设置 max_buf_size_
  if (!option_tools::safeGet(streamer_node, "buffer_length", &max_buf_size_)) {
    LOG(INFO) << tag_ << ": Unable to load buffer length! Using default instead.";
    max_buf_size_ = 32768;
  }
  // 如果 input、logging、output 都没有，释放空间并 LOG(FATAL) 报错退出程序
  if ((has_input_ && 
    !(buf_input_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ||
    (has_logging_ && 
    !(buf_logging_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ||
    (has_output_ && 
    !(buf_output_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ) {
    free(buf_input_); free(buf_logging_); free(buf_output_);
    LOG(FATAL) << __FUNCTION__ << ": Buffer malloc error!";
  }

  // Get loop rate
  if (!option_tools::safeGet(streamer_node, "loop_duration", &loop_duration_)) {
    LOG(INFO) << tag_ << ": Unable to load loop duration! Using default instead.";
    loop_duration_ = 0.005;
  }

  // 获取配置选项中 start_message 到 message
  // Get messages to send on start-up 
  std::string message;
  option_tools::safeGet(streamer_node, "start_message", &message);
  if (message.size() > 0 && !has_input_) {
    LOG(ERROR) << tag_ << ": The start_message options only works with input streams!";
    return;
  }

  // 根据数据流读写模式（R\W\RW）调用成员函数 open 开启数据流
  // Open streamer
  StreamerRWType rw;
  if (has_logging_ || has_output_) rw = StreamerRWType::Write;
  if (has_input_) rw = StreamerRWType::Read;
  if ((has_logging_ || has_output_ || !message.empty()) && has_input_) {
    rw = StreamerRWType::ReadAndWrite;
  }
  if (!streamer_->open(rw)) {
    LOG(ERROR) << "Open streamer " << tag_ << " failed!";
  }
  else opened_ = true;

  // 把 message 转为 uint8_t，调用成员函数 write 发数据
  // Send start-up message
  if (!message.empty()) {
    uint8_t *message_buf = (uint8_t *)malloc(sizeof(uint8_t) * message.size());
    memcpy(message_buf, message.data(), message.size());
    streamer_->write(message_buf, message.size());
    free(message_buf);
  }

  // Set valid 如果前面执行都顺利，valid_ 字段设为 true
  valid_ = true;

  // Save to global for binding
  static_this_.push_back(this);
}
```

#### 2. Streaming::run()：Streaming 线程执行的函数

整个 Streaming 线程是一个 while 死循环，即 spin 状态，每隔 loop_duration_ 时间，判断一次 `has_input_`、`has_logging_`、`has_output_` 标志。

* 如果 has_input_，调用 processInput() 处理输入。
* 如果 has_logging_，调用 processLogging() 进行日志输出。
* 如果 has_output_，调用 processOutput() 进行输出。

```cpp
void Streaming::run()
{
  // 直到发出 quit 命令或全局退出，Streaming进程才退出
  // Spin until quit command or global shutdown called 
  SpinControl spin(loop_duration_);
  while (!quit_thread_ && SpinControl::ok()) {
    if (has_input_) {
      processInput(); // 处理输入数据
    }

    if (has_logging_) {
      mutex_logging_.lock();
      processLogging(); // 处理输出日志
      mutex_logging_.unlock();
    }

    if (has_output_) {
      mutex_output_.lock();
      processOutput();  // 处理输出数据
      mutex_output_.unlock();
    }

    spin.sleep();
  }
}
```

#### 3. Streaming::processInput()：处理输入数据

* 调用 streamer 的 read 函数读取数据到 buf_input_。
* 找到对应 formator，调用 formator 的 decode 函数，将 buf_input_ 解码到 dataset。
* 找到对应数据回调函数，执行。
* 日志记录。

```cpp
void Streaming::processInput()
{
  // 调用 streamer 的 read 函数读取数据到 buf_input_
  // Read data from stream  
  buf_size_input_ = streamer_->read(buf_input_, max_buf_size_);
  if (buf_size_input_ == 0) return;

  
  // Decode stream 
  // 成员变量 formators_进 行遍历,定义对应索引的 data_clusters_[i] 的引用 dataset,之后当前 format 解码得到的数据就会存放在这里
  for (size_t i = 0; i < formators_.size(); i++) {
    if (formators_[i].type != StreamIOType::Input) continue;
    std::shared_ptr<FormatorBase>& formator = formators_[i].formator;
    std::vector<std::shared_ptr<DataCluster>>& dataset = data_clusters_[i];
    
    // 调用 formator 的 decode 函数，将 buf_input_ 解码到 dataset
    // decode函数是在FormatBase基类中声明的虚函数,针对不同的format类型，有着不同的解码函数
    // 解码之后的数据就会放在dataset里，返回得到的nobs应该就是观测值(数据)的个数
    int nobs = formator->decode(buf_input_, buf_size_input_, dataset);

    // Call convertion callbacks
    for (int iobs = 0; iobs < nobs; iobs++) {
      
      // Call data callback  调用 data_callback 
      if (data_callbacks_.size() > 0) 
        for (auto it : data_callbacks_) {
        auto& data_callback = it;
        data_callback(formators_[i].tag, dataset[iobs]);
      }

      // Call logger pipeline  调用 pipeline 传日志
      auto it_i = pipelines_convert_.find(formators_[i].tag);
      if (it_i == pipelines_convert_.end()) continue;
      if (it_i->second.size() == 0) continue;
      auto& pipelines = it_i->second;
      for (auto it_j : pipelines) {
        auto& pipeline = it_j.second;
        pipeline(formators_[i].tag, dataset[iobs]);
      }
    }
  }

  // Call direct pipeline
  for (auto it : pipelines_direct_) {
    auto& pipeline = it.second;
    pipeline(buf_input_, buf_size_input_);
  }
}
```

#### 4. processLogging()：处理日志输出

先判断 `need_logging_`，再调用 `streamer` 的 `write` 函数，将 `buf_logging_` 输出

```cpp
// Stream logging processing
void Streaming::processLogging()
{
  if (!need_logging_) return;

  streamer_->write(buf_logging_, buf_size_logging_);

  buf_size_logging_ = 0;
  need_logging_ = false;
}
```

#### 5. processOutput()：处理输出数据

 先判断 `need_output_`，再调用 `streamer` 的 `write` 函数，将 `buf_output_` 输出 

```cpp
void Streaming::processOutput()
{
  if (!need_output_) return;

  streamer_->write(buf_output_, buf_size_output_);

  buf_size_output_ = 0;
  need_output_ = false;
}
```

### 6、Estimating 线程

#### 1. EstimatingBase 构造函数

获取到 `estimator` 节点一些基本的配置项 

```cpp
EstimatingBase::EstimatingBase(
  const NodeOptionHandlePtr& nodes, size_t i_estimator) : 
  compute_covariance_(true)
{
  // Get options 根据传入的 estimator_node 索引从nodes 中取出对应的 estimator 配置选项到 node、estimator_node
  const auto& estimator_node = nodes->estimators[i_estimator];  
  const YAML::Node& node = estimator_node->this_node;
  tag_ = estimator_node->tag;                   // 获取 tag、type
  std::string type_str = estimator_node->type;
  option_tools::convert(type_str, type_);

  if (!option_tools::safeGet(
      node, "output_align_tag", &output_align_tag_)) {
    LOG(FATAL) << "Unable to load estimator loop duration align tag!";
    return;
  }
  if (output_align_tag_.substr(0, 4) == "fmt_" || 
      output_align_tag_.substr(0, 4) == "str_" || 
      output_align_tag_.substr(0, 4) == "est_") {
    output_align_tag_ = 
      output_align_tag_.substr(4, output_align_tag_.size() - 4);
  }

  if (!option_tools::safeGet(node, "compute_covariance", &compute_covariance_)) {
    LOG(INFO) << "Unable to load compute_covariance!";
  }
  // input data roles 
  for (size_t i = 0; i < estimator_node->input_tags.size(); i++) {
    const std::string& tag = estimator_node->input_tags[i];
    const std::vector<std::string>& roles = estimator_node->input_tag_roles[i];
    if (tag.substr(0, 4) != "est_") continue;
    CHECK(roles.size() == 1);
    SolutionRole solution_role;
    option_tools::convert(roles[0], solution_role);
    estimator_tag_to_role_.insert(std::make_pair(tag, solution_role));
  } 

  // input data alignment
  if (option_tools::safeGet(node, "enable_input_align", &enable_input_align_)) {
    if (enable_input_align_)
    if (!option_tools::safeGet(node, "input_align_latency", &input_align_latency_)) {
      LOG(FATAL) << "Unable to load input_align_latency while enable_input_align is setted!";
      return;
    }
  }

  // backend pending check and measurement data sparsifying
  if (option_tools::safeGet(node, 
    "enable_backend_data_sparsify", &enable_backend_data_sparsify_)) {
    if (enable_backend_data_sparsify_)
    if (!option_tools::safeGet(node, 
      "pending_num_threshold", &pending_num_threshold_)) {
      LOG(FATAL) << "Unable to load pending_num_threshold "
                 << "while enable_backend_data_sparsify is setted!";
      return;
    }
  }
```

#### 2. MultiSensorEstimating() 构造函数

- 先调用父类 `EstimatingBase` 的构造函数，获取到 `estimator` 节点一些基本的配置项。
- 加载估计器基础选项，赋值对应字段。
- 加载 GNSS、IMU、Camera 选项，根据传感器类型和定位模式，创建对应的 estimator。
- 创建 SPP estimator，因为所有解算所有模式都要用 SPP。
- 结果时间戳设为 0.0 

```cpp
MultiSensorEstimating::MultiSensorEstimating(
  const NodeOptionHandlePtr& nodes, size_t i_estimator) : 
  EstimatingBase(nodes, i_estimator), latest_imu_timestamp_(0.0)
{
  // load base options
  const YAML::Node& node = nodes->estimators[i_estimator]->this_node;
  YAML::Node estimator_base_node = node["estimator_base_options"];
  if (estimator_base_node.IsDefined()) {
    option_tools::loadOptions(estimator_base_node, base_options_);
  }

  force_initial_global_position_ = base_options_.force_initial_global_position;
  initial_global_position_ = base_options_.initial_global_position;
  base_options_.compute_covariance = compute_covariance_;

  // load GNSS base options
  YAML::Node gnss_base_node = node["gnss_estimator_base_options"];
  YAML::Node gnss_loose_base_node = node["gnss_loose_estimator_base_options"];
  if (estimatorTypeContains(SensorType::GNSS, type_) && 
      gnss_base_node.IsDefined()) {
    option_tools::loadOptions(gnss_base_node, gnss_base_options_);
  }
  if (estimatorTypeContains(SensorType::GNSS, type_) && 
      gnss_loose_base_node.IsDefined()) {
     option_tools::loadOptions(gnss_loose_base_node, gnss_loose_base_options_);
  }

  // load IMU base options
  YAML::Node imu_base_node = node["imu_estimator_base_options"];
  if (estimatorTypeContains(SensorType::IMU, type_) && 
      imu_base_node.IsDefined()) {
    option_tools::loadOptions(imu_base_node, imu_base_options_);
  }

  // load camera base options
  YAML::Node visual_estimator_base_node = node["visual_estimator_base_options"];
  YAML::Node feature_handler_node = node["feature_handler_options"];
  if (estimatorTypeContains(SensorType::Camera, type_) && 
      visual_estimator_base_node.IsDefined()) {
    option_tools::loadOptions(visual_estimator_base_node, visual_estimator_base_options_);
  }
  if (estimatorTypeContains(SensorType::Camera, type_) && 
      feature_handler_node.IsDefined()) {
    option_tools::loadOptions(feature_handler_node, feature_handler_options_);
  }

  // Instantiate estimators
  // single point positioning （GNSS-SPP）
  if (type_ == EstimatorType::Spp) 
  {
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    estimator_.reset(new SppEstimator(
      spp_options_, gnss_base_options_, base_options_));
  }
  // single-differenced GNSS pseudorange positioning （GNSS-SD)
  else if (type_ == EstimatorType::Sdgnss) 
  {
    YAML::Node sdgnss_node = node["sdgnss_options"];
    if (sdgnss_node.IsDefined()) {
      option_tools::loadOptions(sdgnss_node, sdgnss_options_);
    }
    estimator_.reset(new SdgnssEstimator(
      sdgnss_options_, gnss_base_options_, base_options_));
  }
  // double-differenced GNSS pseudorange positioning (differential GNSS) （GNSS-RTD)
  else if (type_ == EstimatorType::Dgnss) 
  {
    YAML::Node dgnss_node = node["dgnss_options"];
    if (dgnss_node.IsDefined()) {
      option_tools::loadOptions(dgnss_node, dgnss_options_);
    }
    estimator_.reset(new DgnssEstimator(
      dgnss_options_, gnss_base_options_, base_options_));
  }
  // real-time kinematic （GNSS-RTK）
  else if (type_ == EstimatorType::Rtk) 
  {
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }
    estimator_.reset(new RtkEstimator(
      rtk_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // precise point positioning （GNSS-PPP)
  else if (type_ == EstimatorType::Ppp) 
  {
    YAML::Node ppp_node = node["ppp_options"];
    if (ppp_node.IsDefined()) {
      option_tools::loadOptions(ppp_node, ppp_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }
    estimator_.reset(new PppEstimator(
      ppp_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU loosely couple （GI-LC)
  else if (type_ == EstimatorType::GnssImuLc)
  {
    YAML::Node gnss_imu_lc_node = node["gnss_imu_lc_options"];
    if (gnss_imu_lc_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_lc_node, gnss_imu_lc_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new GnssImuLcEstimator(
      gnss_imu_lc_options_, gnss_imu_init_options_, gnss_loose_base_options_, 
      imu_base_options_, base_options_));
  }
  // SPP/IMU tightly couple （SPP/IMU-TC)
  else if (type_ == EstimatorType::SppImuTc)
  {
    YAML::Node spp_imu_tc_node = node["spp_imu_tc_options"];
    if (spp_imu_tc_node.IsDefined()) {
      option_tools::loadOptions(spp_imu_tc_node, spp_imu_tc_options_);
    }
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new SppImuTcEstimator(
      spp_imu_tc_options_, gnss_imu_init_options_, spp_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_));
  }
  // RTK/IMU tightly couple （RTK/IMU-TC)
  else if (type_ == EstimatorType::RtkImuTc)
  {
    YAML::Node rtk_imu_tc_node = node["rtk_imu_tc_options"];
    if (rtk_imu_tc_node.IsDefined()) {
      option_tools::loadOptions(rtk_imu_tc_node, rtk_imu_tc_options_);
    }
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new RtkImuTcEstimator(
      rtk_imu_tc_options_, gnss_imu_init_options_, rtk_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU/Camera semi-tightly integration (GIC-STC)
  else if (type_ == EstimatorType::GnssImuCameraSrr)
  {
    YAML::Node gnss_imu_camera_srr_node = node["gnss_imu_camera_srr_options"];
    if (gnss_imu_camera_srr_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_camera_srr_node, gnss_imu_camera_srr_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new GnssImuCameraSrrEstimator(gnss_imu_camera_srr_options_,
      gnss_imu_init_options_, gnss_loose_base_options_, visual_estimator_base_options_, 
      imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // SPP/IMU/Camera tightly integration (SPP/IMU/Camera-TC)
  else if (type_ == EstimatorType::SppImuCameraRrr)
  {
    YAML::Node spp_imu_camera_rrr_node = node["spp_imu_camera_rrr_options"];
    if (spp_imu_camera_rrr_node.IsDefined()) {
      option_tools::loadOptions(spp_imu_camera_rrr_node, spp_imu_camera_rrr_options_);
    }
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics 旋转外参
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new SppImuCameraRrrEstimator(spp_imu_camera_rrr_options_, 
      gnss_imu_init_options_, spp_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // RTK/IMU/Camera tightly integration (RTK/IMU/Camera-TC)
  else if (type_ == EstimatorType::RtkImuCameraRrr)
  {
    YAML::Node rtk_imu_camera_rrr_node = node["rtk_imu_camera_rrr_options"];
    if (rtk_imu_camera_rrr_node.IsDefined()) {
      option_tools::loadOptions(rtk_imu_camera_rrr_node, rtk_imu_camera_rrr_options_);
    }
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new RtkImuCameraRrrEstimator(rtk_imu_camera_rrr_options_, 
      gnss_imu_init_options_, rtk_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_, ambiguity_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  else {
    LOG(ERROR) << "Invalid estimator type: " << static_cast<int>(type_);
    return;
  }

  // 创建 SPP 估计器，因为所有解算所有模式都要用 SPP
  // For coordinate initialization  
  if (estimatorTypeContains(SensorType::GNSS, type_)) {
    spp_estimator_.reset(new SppEstimator(gnss_base_options_)); 
  }

  // Initial values
  solution_.timestamp = 0.0;  // 结果时间戳设为 0.0
}
```

#### 3. MultiSensorEstimating::process()：创建 frontend、MeasurementAddin、Backend 线程

```cpp
void MultiSensorEstimating::process()
{
  // estimatorTypeContains 检查如果有 Camera，
  // Process frontends in a separated thread 
  if (image_frontend_thread_ == nullptr && 
      estimatorTypeContains(SensorType::Camera, type_)) {
    image_frontend_thread_.reset(new std::thread(
      &MultiSensorEstimating::runImageFrontend, this)); // frontend 线程
  }

  // Put measurements from addin buffer to measurement buffer
  if (measurement_thread_ == nullptr) {
    measurement_thread_.reset(new std::thread(
      &MultiSensorEstimating::runMeasurementAddin, this)); // export 线程
  }

  // Process backend in a separated thread
  if (backend_thread_ == nullptr) {
    backend_thread_.reset(new std::thread(
      &MultiSensorEstimating::runBackend, this)); // backend 线程
  }

  // kill threads 如果外部有设置终止线程，就把线程都终止了
  if (quit_thread_) {
    if (image_frontend_thread_) {
      image_frontend_thread_->join(); image_frontend_thread_ = nullptr;
    }
    if (measurement_thread_) {
      measurement_thread_->join(); measurement_thread_ = nullptr;
    }
    if (backend_thread_) {
      backend_thread_->join(); backend_thread_ = nullptr;
    }
  }
}
```

#### 4. MultiSensorEstimating::runImageFrontend()：视觉前端线程

整个视觉前端线程是一个 while 死循环，即 spin 状态，每隔 1.0e-4 时间，执行一次循环。

* 循环开始先检查有没有新 image 数据，然后判断相机类型，只处理单目，其它跳过。
* 检查时间戳是否有效，当前帧不能早于上一帧。
* 判断数据量大小，如果图像超过 5 个且大于之前最大值会进行提示。
* 把图像转为 CV::Mat，获取时间戳、标签，尝试获取当前帧的位姿。
* 调用 addImageBundle() 添加量测值，并判断能否解算，如果可以处理，调用 processImageBundle() 特征提取追踪。
* 如果特征提取追踪成功，执行 estimatorDataCallback() 回调；ROS 模式根据特征点绘图

```cpp
void MultiSensorEstimating::runImageFrontend()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    // Check if we have new image data 检查有没有新 image 数据
    mutex_image_input_.lock();
    if (image_frontend_measurements_.size() == 0) {
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }
    EstimatorDataCluster& front_measurement = image_frontend_measurements_.front();

    // 判断相机类型，只处理单目，其它跳过
    if (front_measurement.image_role != CameraRole::Mono) {
      image_frontend_measurements_.pop_front();
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }

    // 检查时间戳是否有效，当前帧不能早于上一帧
    // Check if timestamp is valid 
    if (!feature_handler_->isFirstFrame() && 
        feature_handler_->getFrameBundle()->getMinTimestampSeconds() >= 
        front_measurement.timestamp) {
      LOG(WARNING) << "Image timestamp descending detected! ("
        << std::fixed << front_measurement.timestamp << " vs " 
        << feature_handler_->getFrameBundle()->getMinTimestampSeconds() << ")";
      image_frontend_measurements_.pop_front();
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }

    // 判断数据量大小，如果图像超过 5 个且大于之前最大值会进行提示
    // Check pending 
    if (image_frontend_measurements_.size() > 5) {
      if (last_image_pending_num_ != image_frontend_measurements_.size()) {
        LOG(WARNING) << "Large image frontend pending: " 
                     << image_frontend_measurements_.size()
                     << " frames are waiting!";
      }
      last_image_pending_num_ = image_frontend_measurements_.size();
    }


    // Process feature detecting and tracking 进行特征提取和跟踪 
    // 把图像转为 CV::Mat，获取时间戳、标签，尝试获取当前帧的位姿
    std::shared_ptr<cv::Mat>& image = front_measurement.image; 
    double timestamp = front_measurement.timestamp;
    std::string tag = front_measurement.tag;
    bool ret = false;
    Transformation T_WS;
    if (!estimator_->getPoseEstimateAt(timestamp, T_WS)) {
      ret = feature_handler_->addImageBundle({image}, timestamp);
    }
    else {
    
      // 调用 addImageBundle() 添加量测值，并判断能否解算
      ret = feature_handler_->addImageBundle({image}, timestamp, {T_WS});
    }
    image_frontend_measurements_.pop_front();
    mutex_image_input_.unlock();

    // 如果可以处理，调用 processImageBundle() 特征提取追踪
    if (ret) {
      ret = feature_handler_->processImageBundle();
    }

    // 如果特征提取追踪成功
    if (ret) {
      FrameBundlePtr frame_bundle = feature_handler_->getFrameBundle();
      EstimatorDataCluster measurement(frame_bundle, tag);
      estimatorDataCallback(measurement);

      // ROS 模式根据特征点绘图
      // call featured image output and map point output (for ROS)
      const FramePtr& frame = frame_bundle->at(0);
      const MapPtr& map = feature_handler_->getMap();
      std::shared_ptr<DataCluster> frame_data = std::make_shared<DataCluster>(frame);
      std::shared_ptr<DataCluster> map_data = std::make_shared<DataCluster>(map);
      for (auto& callback : output_data_callbacks_) {
        callback(tag_, frame_data);
        callback(tag_, map_data);
      }
    }
    spin.sleep();
  }
}
```

#### 5. runMeasurementAddin()、putMeasurements()：量测添加线程

整个量测添加线程是一个 while 死循环，即 spin 状态，每隔 1.0e-4 时间执行一次 putMeasurements()：

* 如果有数据， 从 measurement_addin_buffer_ 容器里获得数据。
* 调用 handleTimePropagationSensors() 向里面加 IMU 数据，作为时间更新的基准。
* 调用 handleFrontendSensors() 向里面加图像数据。
* 不做时间校准直接加 GNSS 基准站数据。
* 调用 handleNonTimePropagationSensors() 把剩下的数据也加进来，不作为时间传播的基准。

```cpp
void MultiSensorEstimating::runMeasurementAddin()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    putMeasurements();
    spin.sleep();
  }
}
```

```cpp
void MultiSensorEstimating::putMeasurements()
{
  // get data 从measurement_addin_buffer_容器里获得数据
  mutex_addin_.lock();
  if (measurement_addin_buffer_.size() == 0) {
    mutex_addin_.unlock(); return;
  }
  EstimatorDataCluster data = measurement_addin_buffer_.front();
  measurement_addin_buffer_.pop_front();
  mutex_addin_.unlock();

  // time-propagation sensors 向里面加 IMU 数据，作为时间更新的基准
  if (estimatorDataIsImu(data)) {
    handleTimePropagationSensors(data);
  } 
  // sensors that needs frontends 向里面加图像数据
  else if (estimatorDataNeedFrontend(data)) {
    handleFrontendSensors(data);
  }
  // GNSS reference station data (no need to align time) 不做时间校准直接加 GNSS 基准站数据
  else if (data.gnss && data.gnss_role == GnssRole::Reference) {
    mutex_input_.lock();
    measurements_.push_back(data);
    mutex_input_.unlock();
  }
  // other sensors  把剩下的数据也加进来
  else {
    handleNonTimePropagationSensors(data); // 不作为时间传播的基准
  }
}
```

##### handleTimePropagationSensors()：添加 IMU 数据

```cpp
// Handle time-propagation sensors
void MultiSensorEstimating::handleTimePropagationSensors(EstimatorDataCluster& data)
{
  // only support IMU 只支持 IMU
  CHECK(estimatorDataIsImu(data));
  if (estimator_) estimator_->addMeasurement(data);
  mutex_input_.lock();
  latest_imu_timestamp_ = data.timestamp; // 将当前数据的时间戳直接设为时间
  mutex_input_.unlock();
  // align timeline for output control 把时间戳加入到output_timestamps_之中
  if (backend_firstly_updated_ && output_align_tag_ == data.tag) {
    if (checkDownsampling(data.tag)) {
      mutex_output_.lock();
      output_timestamps_.push_back(data.timestamp); 
      mutex_output_.unlock();
    }
  }
}
```

##### handleFrontendSensors()：添加图像数据

```cpp
// Handle sensors that need frontends
void MultiSensorEstimating::handleFrontendSensors(EstimatorDataCluster& data)
{
  CHECK(estimatorDataNeedFrontend(data));
  if (data.image) {
    mutex_image_input_.lock();
    image_frontend_measurements_.push_back(data);
    mutex_image_input_.unlock();
  }
}
```

##### handleNonTimePropagationSensors()：添加其它传感器数据（GNSS）、稀疏化

```cpp
void MultiSensorEstimating::handleNonTimePropagationSensors(EstimatorDataCluster& data)
{
  // 
  // Input align mode
  if (enable_input_align_) { // 判断的依据是根据配置文件中的 enable input align
    // Insert a measurement to addin buffer realigning timestamps
    const double buffer_time = 2.0 * input_align_latency_;
    if (!needTimeAlign(type_)) {  // 判断是否需要校准，只有 GNSS/IMU/Camera 三种传感器组合的时候需要进行校准
      measurement_align_buffer_.push_back(data);
    }
    else if (measurement_align_buffer_.size() == 0 ||  
        data.timestamp >= measurement_align_buffer_.back().timestamp) {
      measurement_align_buffer_.push_back(data);  // 没有数据或者当前数据很新的时候也会加入容器
    }
    else if (data.timestamp <= measurement_align_buffer_.front().timestamp) {
      if (data.timestamp < measurement_align_buffer_.back().timestamp - buffer_time) {
        LOG(WARNING) << "Throughing data at timestamp " << std::fixed << data.timestamp 
          << " because its latency is too large!";
      }
      else {
        measurement_align_buffer_.push_front(data);   // 如果比最老的时间还老的话，根据阈值判断，满足阈值要求就加进去
      }
    }
    else
    for (auto it = measurement_align_buffer_.begin();  
          it != measurement_align_buffer_.end(); it++) {
      if (data.timestamp >= it->timestamp) continue;
      measurement_align_buffer_.insert(it, data); // 都不满足的话，就找到合适的位置插进去
      break;
    }
  }
  // Non-align mode
  else {
    measurement_align_buffer_.push_back(data);
  }

  // Check if we can add to measurement buffer 判断要不要向measurements_里加入数据
  // 对measurement_align_buffer_里的数据进行遍历
  for (auto it = measurement_align_buffer_.begin(); it != measurement_align_buffer_.end();)
  {
    // we delay the data for input_align_latency_ to wait incoming data for realigning.
    if (measurement_align_buffer_.back().timestamp - 
        measurement_align_buffer_.front().timestamp < input_align_latency_) break;  // 如果时间太短，就直接break

    // we always add IMU measurement to estimator at a given timestamp before we 
    // add other sensor measurements.
    // 在估计的类型包括IMU但当前的时间戳比最新的时间戳还新的时候，相当于没有一个校准的基准，这个时候就直接跳出了
    EstimatorDataCluster& measurement = *it;
    if (estimatorTypeContains(SensorType::IMU, type_) && 
        measurement.timestamp > latest_imu_timestamp_) {
      it++; continue;
    }

    mutex_input_.lock();

    // add measurements 满足条件后就把数据加进去
    measurements_.push_back(measurement);

    // check pending, sparcify if needed
    // 根据配置文件中的 enable_backend_data_sparsify
    // 来根据后端是否还在等待处理，判断是否要进行数据的稀疏化
    if (enable_backend_data_sparsify_)  
    {
      if (measurements_.size() > pending_num_threshold_) {
        pending_sparsify_num_++;  // 等待稀疏的个数+1
      }
      else if (pending_sparsify_num_ > 0) pending_sparsify_num_--; // 比阈值小但是有等待稀疏的话，就减少一个
      if (pending_sparsify_num_) {  // 这里执行具体的稀疏数据操作
        LOG(WARNING) << "Backend pending! Sparsifying measurements with counter " 
                    << pending_sparsify_num_ << ".";
        for (int i = 0; i < pending_sparsify_num_; i++) {
          // some measurements we cannot erase
          if (measurements_.front().frame_bundle && 
              measurements_.front().frame_bundle->isKeyframe()) break;  // 关键帧留一下
          // erase front measurement
          if (measurements_.size() > 1) measurements_.pop_front();  // 不然就从前面剔除
        }
      }
    }

    mutex_input_.unlock();
    it = measurement_align_buffer_.erase(it);
  }
}
```

#### 6. runBackend、processEstimator：定位解算

* 检测有没有要处理的数据，没有数据直接返回等下一次再检查。
* 如果有数据，观测值队列取出并删除最前面的数据。观测量队列数据超过 5，且和之前的数量不一致就报警告。
* 设置初始坐标和重力，有以下几种方式，如果得不到初始坐标和重力，直接返回。
  * 手动设置初始坐标
  * 或调用addMeasurement、estimate 求出 SPP 解作为初始坐标
  * 或根据收到的 GNSS 结果作为初始坐标
  * 有 IMU，调用 earthGravity 获取重力
* 调用 addMeasurement() 向正式估计器因子图加参数块。
* 调用 estimate() 进行因子图优化估计，并取出估计的结果。
* 调用 getStatus() 检查估计状态，如果是 Diverged 估计发散，就调用 resetProcessors() 重置估计器。
* 调用 logIntermediateData() 记录中间结果。

```cpp
void MultiSensorEstimating::runBackend()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    processEstimator();
    spin.sleep();
  }
}
```

```cpp
bool MultiSensorEstimating::processEstimator()
{
  // Check if we have data to process 检测有没有要处理的数据
  mutex_input_.lock();
  if (measurements_.size() == 0) {
    mutex_input_.unlock(); return false;
  }

  // 从 measurements_ 观测值队列取出并删除最前面的数据
  EstimatorDataCluster measurement = measurements_.front();
  measurements_.pop_front();
  mutex_input_.unlock();

  // Check pending 如果 measurements_ 观测量队列数据超过 5，且和之前的数量不一致就报警告
  if (measurements_.size() > 5) {
    if (last_backend_pending_num_ != measurements_.size()) {
      LOG(WARNING) << "Large backend pending: " << measurements_.size()
                  << " measurements are waiting!";
    }
    last_backend_pending_num_ = measurements_.size();
  }


  // Set coordinate and gravity 
  if (solution_.coordinate == nullptr) // 如果还没有 solution_.coordinate
  {
    Eigen::Vector3d position_ecef;

    // Force set coordinate zero 手动设置初始坐标
    if (force_initial_global_position_) { 
      GeoCoordinate coordinate;
      position_ecef = coordinate.convert(
        GeoCoordinate::degToRad(initial_global_position_), 
        GeoType::LLA, GeoType::ECEF);
    }
    // get coordinate zero from GNSS raw 根据原始观测值求出 SPP 解作为初始坐标
    else if (measurement.gnss && 
             estimatorTypeContains(SensorType::GNSS, type_)) {
      if (!spp_estimator_->addMeasurement(measurement)) {   // addMeasurement
        return false;
      }
      if (!spp_estimator_->estimate()) {    // estimate
        return false;
      }
      position_ecef = spp_estimator_->getPositionEstimate();  // getPositionEstimate
      measurement.gnss->position = position_ecef;
    }
    // get coordinate zero from solution 
    else if (measurement.solution) {
      position_ecef = measurement.solution->coordinate->convert(
        measurement.solution->pose.getPosition(), 
        GeoType::ENU, GeoType::ECEF);
    }
    // no where to get coordinate zero
    else {
      return false;
    }

    // set coordinate
    solution_.coordinate = std::make_shared<GeoCoordinate>(
      position_ecef, GeoType::ECEF);
    Eigen::Vector3d lla = solution_.coordinate->convert(
      position_ecef, GeoType::ECEF, GeoType::LLA);
    estimator_->setCoordinate(solution_.coordinate);
    
    // Set gravity if needed
    if (estimatorTypeContains(SensorType::IMU, type_)) {
      double gravity = earthGravity(lla);
      std::shared_ptr<ImuEstimatorBase> imu_estimator = 
        std::dynamic_pointer_cast<ImuEstimatorBase>(estimator_);
      CHECK_NOTNULL(imu_estimator);
      imu_estimator->setGravity(gravity);
    }
  }

  // Process estimator 正式估计
  bool is_updated = false;

  // add measurement
  if (estimator_->addMeasurement(measurement)) {    // addMeasurement
    // solve
    if (estimator_->estimate()) is_updated = true;  // estimate
    // check if estimator valid
    if (estimator_->getStatus() == EstimatorStatus::Diverged) { // getStatus
      // reset estimator
      LOG(WARNING) << "Reset estimator because it is diverge!";
      resetProcessors();  // resetProcessors
      is_updated = false;
    }
    // log intermediate data
    estimator_->logIntermediateData();  // logIntermediateData
  }

  // Backend updated 更新结果和初始状态
  if (is_updated)
  {
    // update flag
    backend_firstly_updated_ = true;
    // align timeline for output control  对齐时间线进行输出控制
    if (output_align_tag_ == measurement.tag) {
      mutex_output_.lock();
      if (checkDownsampling(measurement.tag)) {   // checkDownsampling
        output_timestamps_.push_back(measurement.timestamp);
      }
      mutex_output_.unlock();
    }
    // check pendding
    if (measurements_.size() > 1) {
      double pending_period = measurements_.back().timestamp - 
                        solution_.timestamp;
      // add feedbacks here
    }
  }

  return true;
}
```

#### 7. updateSolution：将结果集成，转化成需要的采样率

```cpp
bool MultiSensorEstimating::updateSolution()
{
  // Backend not working yet 确保已经开启了解算
  if (!backend_firstly_updated_) return false;

  mutex_output_.lock();

  // No need to update 确保有需要更新的解算结果
  if (output_timestamps_.size() == 0) {
    mutex_output_.unlock(); return false;
  }

  // Erase timestamps in front of estimator window
  while (output_timestamps_.front() < estimator_->getOldestTimestamp()) {
    LOG(WARNING) << "Erasing output timestamp " << std::fixed 
      << output_timestamps_.front() << " because it is too old!";
    output_timestamps_.pop_front();
  }

  // Check pending  待处理的时间差判断，要是大于1了就提示待处理的太多了
  if (output_timestamps_.back() - output_timestamps_.front() > 1.0) {
    LOG(WARNING) << "Large pending in output control!";
  }

  // Get solution 
  const double timestamp = output_timestamps_.front();
  mutex_output_.unlock();
  solution_.timestamp = timestamp;
  solution_.covariance.setZero();
  if (!estimator_->getPoseEstimateAt(timestamp, solution_.pose) || 
      !estimator_->getSpeedAndBiasEstimateAt(timestamp, solution_.speed_and_bias) || 
      (compute_covariance_ && 
      !estimator_->getCovarianceAt(timestamp, solution_.covariance))) {
    return false;
  }

  // if we have GNSS, get GNSS variables
  if (estimatorTypeContains(SensorType::GNSS, type_)) {
    if (std::shared_ptr<GnssEstimatorBase> gnss_estimator = 
      std::dynamic_pointer_cast<GnssEstimatorBase>(estimator_)) {
      solution_.status = gnss_estimator->getSolutionStatus();             // 解的状态
      solution_.num_satellites = gnss_estimator->getNumberSatellite();    // 解的状态
      solution_.differential_age = gnss_estimator->getDifferentialAge();  // 解的状态
    }
    else if (std::shared_ptr<GnssLooseEstimatorBase> gnss_estimator = 
      std::dynamic_pointer_cast<GnssLooseEstimatorBase>(estimator_)) {
      solution_.status = gnss_estimator->getSolutionStatus();
      solution_.num_satellites = gnss_estimator->getNumberSatellite();
      solution_.differential_age = gnss_estimator->getDifferentialAge();
    }
    else {
      LOG(FATAL) << "Unable to cast estimaotr to GNSS estimator!";
    }
  }

  mutex_output_.lock();
  output_timestamps_.pop_front();
  mutex_output_.unlock();

  return true;
}
```

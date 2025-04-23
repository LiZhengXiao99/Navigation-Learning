> 最后修改：2025-01-14，李郑骁

[toc]

## 一、我的 ROS 入门学习路径

**ROS（Robot Operating System）** 是一个开源的机器人开发框架，旨在为机器人软件开发提供通用的工具和库。尽管名字中包含“操作系统”，但 ROS 并不是传统意义上的操作系统，而是一个运行在 Linux 上的中间件（Middleware），提供了硬件抽象、设备驱动、通信机制、工具和算法库等功能；ROS 系统和生态很庞大，涵盖了从底层硬件驱动到高层算法实现的完整机器人开发工具链。

**ROS 的核心是一个分布式的通信框架**，支持多种通信机制，如话题、服务和动作，使得不同模块可以高效地协同工作。其丰富的功能包和工具库覆盖了传感器数据处理、运动控制、导航规划、SLAM（同步定位与地图构建）、人机交互等多个领域，开发者可以根据需求灵活选择和组合这些功能。ROS 的仿真工具（如 Gazebo 和 RViz）为算法开发和测试提供了强大的支持，能够在虚拟环境中验证机器人的行为，减少对物理硬件的依赖。此外，ROS 的开源社区非常活跃，提供了大量的教程、文档和开源项目，极大地降低了学习和开发的门槛。无论是学术研究、工业应用还是教育实践，ROS 都因其模块化、可扩展和跨平台的特点，成为机器人开发的首选框架。通过 ROS，开发者可以快速构建从简单到复杂的机器人系统，实现从算法设计到实际部署的全流程开发。

在 ROS（Robot Operating System）中，**节点**、**功能包**、和 **通信机制** 是构建机器人控制系统的核心要素。节点是 ROS 中的基本执行单元，每个节点是一个独立的进程，负责完成特定的任务，例如传感器数据采集、运动控制或算法处理。功能包是 ROS 中的基本软件单元，用于组织和管理相关的节点、程序、配置文件、启动文件等资源，每个功能包通常实现一个特定的功能模块，例如导航、定位或控制。工作空间是 ROS 中用于开发和存放功能包的目录结构，包含源代码、编译文件和安装文件，开发者可以在工作空间中创建、编译和管理多个功能包。节点之间通过 ROS 的通信机制（如话题、服务、动作）进行数据交换，话题用于异步通信，服务用于同步通信，而动作用于处理长时间运行的任务。

**一套基于 ROS 的机器人控制系统通过工作空间中的多个功能包组合实现，每个功能包包含一个或多个节点，节点之间通过通信机制协同工作**。例如，一个移动机器人系统可能包括激光雷达数据处理的节点、SLAM 算法的节点、路径规划的节点以及电机控制的节点，这些节点分布在不同的功能包中，通过话题传递传感器数据和控制指令，通过服务调用特定的功能，通过动作处理复杂的任务。**启动文件**（`.launch` 文件）用于批量启动这些节点，并配置参数和通信关系，最终形成一个模块化、可扩展的机器人控制系统。通过这种设计，开发者可以在工作空间中灵活地组合和扩展功能，快速构建复杂的机器人应用，例如自主导航、环境感知和任务执行。

![image-20250112204159482](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112204159482.png)

![image-20250112204016798](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112204016798.png)

由于我们只需要开发单独的 GNSS/INS 定位的功能包或者节点，了解 ROS 核心概念并且能运用通信机制既可，无需深入学习全套的机器人控制框架（比如底盘控制、导航、建图、机械臂、Gazebo 仿真），以下是我总结的 ROS 入门路径：

1. 先看书看视频教程了解 ROS 系统的核心概念

   * **ROS 架构**：节点（Node）、功能包（Package）、话题（Topic）、服务（Service）、消息（Message）、参数服务器（Parameter Server）等；

   * **通信机制**：发布/订阅模式（Publisher/Subscriber）和请求/响应模式（Service/Client）；

   * **工具**： roscore、rosrun、roslaunch、rostopic、rosservice、rosnode、rqt、rviz、gaze等；

   * **ROSbag**：记录传感器数据，测试和调试算法；

   > 我先看的是古月的ROS入门21讲，然后看张万杰的视频教程《[机器人操作系统 ROS 快速入门教程](https://www.bilibili.com/video/BV1BP4y1o7pw)》以及配套的图书《机器人操作系统（ROS）及仿真应用》，视频看到17P（共77p），书看二三四章（共九章）。

2. 配置 Ubuntu20 + ROS Noetic + VSCode 开发环境；

3. 通过一些示例来初步学习 ROS 项目的编译、运行；

4. 从零开始创建 ROS 项目，尝试编写一套多节点的 ROS 功能包 Demo；
5. 在阅读了开源 ROS 工程和自己写过 Demo 的基础上，总结 ROS C++ 项目开发流程和代码编写套路。

---

## 二、VM + ROS + VSCode 开发环境搭建

在开发中，VM（虚拟机）+ ROS（机器人操作系统）+ VSCode（Visual Studio Code）是一种常见的开发环境配置。首先，虚拟机（如VMware或VirtualBox）提供了一个隔离的操作系统环境，便于在不同平台上运行 ROS，尤其是当主机系统（WIndows）不兼容 ROS 时。通过 VM 运行 Linux 系统（Ubuntu），安装 ROS 后，开发者可以在 VSCode 中编写和调试 ROS 节点，利用虚拟机与主机的文件共享功能，方便代码管理和测试。

### 1、安装 VM + Ubuntu20

安装过程略，写几个注意事项：

* 磁盘空间默认20G，肯定不够用，虚拟机磁盘可以扩容，但是扩完容还需要在 Ubuntu 内修改分区大小；所以最好在创建虚拟机的时候就设置大一些，选择不立即分配所有磁盘空间。

* 虚拟机与物理机无法进行文本的复制粘贴，可能是 vm-tools 没安装好，用命令安装：

  ```
  sudo apt install open-vm-tools-desktop
  ```

  然后重新启动系统再试试。

* 默认的自动息屏时间太短太频繁了，在电源选项中关闭自动黑屏：

  <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250114090617493.png" alt="image-20250114090617493" style="zoom:45%;" />
  
* 默认设置下的虚拟机可能卡的没法用，这里我收集的一些针对 VM 虚拟机卡顿的优化策略：

  * 设置以管理员身份运行VM：找到 VM 桌面图标，右键设置属性，在“兼容性”栏目中勾选“以管理员身份运行此程序”；
  * 安装时将虚拟磁盘存储为单个文件；
  * 安装 “Minimal installation” 版的 Ubuntu，其中需要注意的几个点：
    * 创建虚拟机的时候选择”暂不安装系统“；
    * 虚拟机设置中 CD/DVD 设置为下载的 Ubuntu 安装包路径；
    * 打开虚拟机的时候进入 Ubuntu 安装，选择 “Minimal installation”。
  * 改 VM 配置：
    * 编辑-首选项-内存：预留内存调大；
    * 编辑-首选项-优先级：抓取输入内容设为高优先级，勾选调整所有虚拟机内存使其适应预留的主机；
    * 虚拟机设置-硬件：内存设置稍微大一些，CPU不要超过物理机个数，硬盘选择 SSCI，把用不着的的光驱和 USB 连接器删了；
    * 虚拟机设置-选项-高级：抓取输入内容设为高优先级，禁用内存页面修整，不收集调试信息；
  * 虚拟机都是文件形式，最好装在速度快的硬盘里；
  * 虚拟机不用的时候不用关机，选暂停，之后启动的更快。

### 2、安装 Ubuntu 基本软件

```bash
sudo apt install <XXX>
```

* **vim**：文本编辑器，可以用于在终端中修改配置文件；
* **tree**：以文件树的形式展示文件结构；
* **docker**：创建和使用镜像，很多编译调试所需环境复杂程序都提供 docker；
* **git**：用来从 Github 上下载程序；
* **net-tools**：安装之后可以使用 ifconfig 查看网络状态；
* **wget**：下载文件，鱼香 ROS 的安装脚本需要用到；
* **gzip**、**tar**：压缩、解压缩；
* **terminator**：支持终端分屏，运行 ROS 命令常常需要用好几个终端；
* **pip**：用来管理 Python 包；
* **ssh**：用来远程连接；

可以直接一行命令安装多个程序：

```
sudo apt install vim tree git docker net-tools wget gzip tar terminator pip ssh
```

### 3、安装 ROS Noetic

使用鱼香ROS 的一键安装脚本，按他的提示，换源安装 ROS：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112143916409.png" alt="image-20250112143916409" style="zoom:50%;" />

脚本能自动将 ROS 启动脚本添加到终端程序初始化的脚本 `~/.bashrc`文件中，这样每次打开终端就可以直接用了：

![image-20250113080747162](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250113080747162.png)

### 4、安装 VSCode 及插件

安装 VSCode 可以从官网下载后缀为 .deb 的安装包，也可以直接在 Ubuntu 的应用商店中安装，还可以用鱼香 ROS 的脚本安装。刚安装的的 VSCode 只是个文本编辑器，要想进行程序开发还得装对应的插件，直接从 VSCode 插件栏可能无法获取插件，可以从 VSCode 的插件应用商店搜索下载 .vsix 插件安装包，然后导入 VSCode：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112091542845.png" alt="image-20250112091542845" style="zoom:50%;" />

如果经常需要在新环境里配置 VSCode，可以把常用的插件都下载到一个文件夹里，直接一次全安装了：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112091745194.png" alt="image-20250112091745194" style="zoom: 67%;" />

设置 VSCode 自动保存文件：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/05444679138c75c63248feb9afbf69f0.png" alt="05444679138c75c63248feb9afbf69f0" style="zoom: 80%;" />

### 5、通过调试 KF-GINS 测试 C++ 和 Python 开发环境

KF-GINS 是武大 i2Nav 实验室开源的一套松组合定位项目，由一套 C++ 松组合解算程序和一个 Python 结果绘制脚本组成，可以读取 IMU 数据文件、GNSS 结果文件，进行松组合解算和结果分析。代码量小，而且项目文件组织的很好，项目已经配置好了一套测试数据， launch.json 和 .yaml 配置文件都不用改，直接可以编译调试，所以我们可以拿来测试 C++ 和 Python 的编译调试环境。

1. 下载 KF-GINS 源码

   ```bash
   git clone https://github.com/i2Nav-WHU/KF-GINS.git
   ```

   > 命令无法下载可以多试两次，还是无法下载可以从网页端下载源码。

2. 用 VSCode 打开 KF-GINS 文件夹

   ```bash
   code ./KF-GINS/
   ```

3. 确保 CMake 项目导入，选择 GCC 工具链

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/e2fe5e59ab2ed18ec03fcd112473c8be.png" alt="e2fe5e59ab2ed18ec03fcd112473c8be" style="zoom: 67%;" /> 

4. 点击底部 Build 编译项目

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f81eab1879931b0f0082ed4f5b609c60.png" alt="f81eab1879931b0f0082ed4f5b609c60" style="zoom: 67%;" />

5. 在左侧的运行调试栏中选择 ”Linux gdb 启动“：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/2165569de8f1fcb34eaae5ba83c3905d.png" alt="2165569de8f1fcb34eaae5ba83c3905d" style="zoom:67%;" />

   因为设置了 stopAtEntry，所以程序调试的时候，会停在主函数的开头：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/fd0424d925bdcc6e1255fb89e2a36044.png" alt="fd0424d925bdcc6e1255fb89e2a36044" style="zoom:67%;" />

6. 接下来可以自行调试程序，也可以不设断点直接点击运行（调试模式运行非常慢）：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/976c4de86124ca8cbbaea47b3c36b18f.png" alt="976c4de86124ca8cbbaea47b3c36b18f" style="zoom:67%;" />

7. 解算完成之后可以用提供的 Python 脚本进行分析，需要先在 launch.json 中添加一个 Python 调试任务，画图脚本只有一个不带参数的 .py 文件，所以选择 Python File：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112160024526.png" alt="image-20250112160024526" style="zoom:67%;" />

   修改 name、cwd、program 设置：

   ![7c2fc4ac16a6ee1095c5603bfaac4633](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7c2fc4ac16a6ee1095c5603bfaac4633.png)

8. Python 脚本用到了 numpy 和 matplot：

   ```bash
   pip install numpy
   pip install matplot
   ```

   ![5cadf72a689bf548f4c8af6503368b09](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5cadf72a689bf548f4c8af6503368b09.png)

9. 选择绘图函数，点击左侧调试选项，运行脚本：

   ![4b6ab481c8073ba99aba9a6160b60b9c](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/4b6ab481c8073ba99aba9a6160b60b9c.png)

   plotNavresult 绘制 PVA：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/dcdc320753dba8604ecab954a4f06923.png" alt="dcdc320753dba8604ecab954a4f06923" style="zoom:67%;" />

   plotNavError 绘制 PVA 误差：

   ![image-20250112162702212](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112162702212.png)

### 6、通过主机中的 MobaXterm 和 VSCode SSH 连接虚拟机进行远程开发

直接在虚拟机中用 VSCode 编写程序可能会比较卡顿，通过主机中的 MobaXterm 和 VSCode SSH 连接虚拟机进行远程开发

1. 虚拟机网络适器设置成 NAT 模式，改完配置要重启虚拟机：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1f2b03e62bc6148f90bf6b790a5addde.png" style="zoom: 67%;" />

2. 安装 ssh：

   ```bash
   sudo apt install openssh-server
   ```

3. 编辑配置文件，开启默认端口号（22）：

   ```bash
   sudo vim /etc/ssh/sshd_config
   ```

   ![e3c27ffa75541b9c520dca4e730a0c71](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/e3c27ffa75541b9c520dca4e730a0c71.png)

4. 重启 ssh 服务：

   ```bash
   sudo service ssh restart
   ```

5. 找到虚拟机的 IP 地址：

   ```bash
   ifconfig
   ```

   ![b130f74c740421c1b2c0c54ce7b3b521](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/b130f74c740421c1b2c0c54ce7b3b521.png)

6. 用主机中的 MobaXterm 连接虚拟机，创建新的 SSH Session，输入IP地址、用户名、端口号，选 OK 进入，刚开始可能要输密码；

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/c3005de8d45da4816370de7387a51f30.png" alt="c3005de8d45da4816370de7387a51f30" style="zoom: 50%;" />

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f0d80514453f39c5f8a724588b4ecb0e.png" alt="f0d80514453f39c5f8a724588b4ecb0e" style="zoom:50%;" />

7. 用主机中的 VSCode 连接虚拟机，需要先在主机的 VSCode 中安装 RemoteSSH 插件：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/0cd6812d1b2e96202924741f36d6c89f.png" alt="0cd6812d1b2e96202924741f36d6c89f" style="zoom:50%;" />

### 7、创建 VM 虚拟机快照

虚拟机（VM）的快照功能允许用户在特定时间点保存系统的完整状态，可以理解为”还原点“。当系统出现故障或配置错误时，可以快速恢复到还原点，避免数据丢失或重新配置的麻烦。可以在刚装好 Ubuntu 系统和配置完基础环境后设置一个还原点，之后开发中如果破坏了环境，不知道怎么改回去，可以回滚到之前的状态。

![b3ec11f23b32cf98c304195cac2723ec](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/b3ec11f23b32cf98c304195cac2723ec.png)

---

## 三、通过几个示例学习 ROS

### 1、以 Turtlesim 为例学习 ROS 基本命令

Turtlesim 是 ROS（机器人操作系统）中的一个经典入门级仿真工具，用于学习和测试 ROS 的基本概念和功能。它模拟了一个简单的二维乌龟机器人，用户可以通过 ROS 话题、服务或动作来控制乌龟的运动，例如移动、旋转或改变其外观。核心节点 `/turtlesim` 负责模拟海龟的行为，并通过话题和服务与其他节点交互。键盘控制节点 `/teleop_turtle` 和 `/turtle_teleop_key` 通过发布速度命令来控制海龟的移动。Turtlesim 提供了一个直观的可视化界面，帮助初学者理解 ROS 的通信机制（如发布/订阅模式）以及节点、话题、服务等核心概念，很多 ROS 初学者入门教程中都以此作为学习 ROS 操作的入门程序。

#### 1. 开启 Turtlesim

分别在三个终端执行下面三个命令：

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

![e84ac1a872f3c0808536396829247101](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/e84ac1a872f3c0808536396829247101.png)

#### 2. 观查 Turtlesim 运行状态

* `rqt_graph`：查看计算图，可以看出系统有两个节点，`/teleop_turtle` 节点通过 `/turtle1/cmd_vel` 话题与 `/turtlesim` 节点通信，来控制乌龟运动：

  <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7328d7cbce540c2ef0997a82fdf3cf84.png" alt="7328d7cbce540c2ef0997a82fdf3cf84" style="zoom: 50%;" />

* `rosnode list`：列出系统中运行的所有节点，可以看出比 rqt_graph 里多出了 `/rosout` 节点，它的作用是采集各个 ROS 节点的日志信息，来提交给界面显示。

  ![df745513343ed7a17625edb0067dd775](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/df745513343ed7a17625edb0067dd775.png)

  `rosnode list` 命令可以配合 grep 来搜索节点，比如 `rosnode list | grep turtle` 搜索所有含 “turtle” 的节点：

  ![5107a92684a7bf3005a2b4d04e5f9b4b](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5107a92684a7bf3005a2b4d04e5f9b4b.png)

* `rosnode info <节点名>`：查看指定节点的信息：

  ![f3da64a42674494bd4dec4950e7c09c5](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f3da64a42674494bd4dec4950e7c09c5.png)

* `rostopic list`：输出系统话题列表：

  ![image-20250112193657946](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250112193657946.png)

* `rosmsg show <消息类型>`：查看指定消息类型的数据结构：

  ![c7acb376b1e476ac43fba1fde7c1c647](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/c7acb376b1e476ac43fba1fde7c1c647.png)

* `rostopic pub <话题名> <消息类型> <消息内容>`：向指定话题中发布指定消息，可以用此命令来控制乌龟移动：

  ![3585fba0f38b5c2995d1e309e452d080](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/3585fba0f38b5c2995d1e309e452d080.png)

* `rosservice list` 列出当前系统中的服务：

  ![76df4ef558317b6d28b15f50a3f37193](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/76df4ef558317b6d28b15f50a3f37193.png)

* `rosservice call`：调用服务，可以调用 `/spawn` 服务，创建一只新的海龟：

  ![1ea820a5254938f5a249807c4e2865cc](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1ea820a5254938f5a249807c4e2865cc.png)

#### 3. 记录操作 Turtlesim 控制指令到 ROSBag

1. 用 `rosbag record` 命令开启记录：

   ```bash
   rosbag record -a -o cmd_record
   ```

2. 上下左右控制海龟移动，Ctrl + C 结束记录；

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/eeb3a99b7bb1f60ac222ba9c9de97bea.png" alt="eeb3a99b7bb1f60ac222ba9c9de97bea" style="zoom:67%;" />

4. 用 `ros play` 命令回放 rosbag 中的控制指令，控制海龟移动：

   ```bash
   rosbag play cmd_record_2025-01-12-20-20-00.bag
   ```

   ![8aaa70a0d7523830ba2e122b72b88e90](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/8aaa70a0d7523830ba2e122b72b88e90.png)

5. 用 `rosbag info` 命令查看 rosbag 信息：

   ![158cbda9fa0af0ae45f2a575a6e98f63](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/158cbda9fa0af0ae45f2a575a6e98f63.png)

### 2、以 wpr_simulation 为例学习 ROS 开源项目的下载编译运行

wpr_simulation 是一个基于 ROS 的仿真项目，主要用于机器人导航、路径规划和环境感知的仿真测试。该项目提供了一个虚拟环境，用户可以在其中测试和验证各种机器人算法，尤其是与移动机器人相关的功能。我们以此来测试 ROS 环境，编译 ROS 项目和 Rviz、Gazebo 的使用。

1. 在合适的位置创建工作空间文件夹

   ```bash
   mkdir -p wpr/src
   ```

2. 进入 src 目录下载源码

   ```bash
   cd wpr/src
   git clone https://github.com/6-robot/wpr_simulation
   ```

3. 运行 scripts 目录下的依赖库安装脚本，安装项目所需的依赖包。

   ```bash
   ./wpr_simulation/scripts/install_for_noetic.sh
   ```

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/676857b1f3a8a6e4ab591e2e04531e5b.png" alt="676857b1f3a8a6e4ab591e2e04531e5b" style="zoom:50%;" />

4. 回到工作空间目录 catkin_make 编译源码

   ```bash
   cd <wpr 目录>
   catkin_make
   ```

5. 加载 devel/setup.bash 到环境参数：

   ```bash
   source devel/setup.bash
   ```

   > 重新打开了终端就需要重新进行这一步操作，所以通常把设置工作空间环境参数的 source 指令添加到终端程序初始化的脚本 `~/.bashrc`文件中，这样每次打开终端就可以直接用了。

6. 输入以下命令启动简单场景仿真：

   ```bash
   roslaunch wpr_simulation wpb_simple.launch
   ```

   在另一个终端输入以下命令打开控制器：

   ```bash
   rosrun rqt_robot_steering rqt_robot_steering
   ```

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250113075434716.png" alt="image-20250113075434716" style="zoom: 67%;" />

   通过 rqt_graph 可以查看系统节点图：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20250113075922988.png" alt="image-20250113075922988" style="zoom:67%;" />

### 3、以 kf-gins-ros 为例学习 VSCode 编译运行调试 ROS 项目

#### 1. 创建工作空间，下载源码

1. 创建工作空间文件夹：

   ```bash
   mkdir -p kf-gins-ros-ws/src
   ```

2. 下载源码到 src 目录：

   ```bash
   cd kf-gins-ros-ws/src
   git clone https://github.com/slender1031/kf-gins-ros.git
   ```

#### 2. 编译运行 data_convert_node、gins_node

1. 修改 `data2bag.cpp` 中的数据文件路径，把 KF-GINS 项目中的 GNSS、INS 数据复制过来：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/e872c72912de048102c96f7c24df5dfc.png" alt="e872c72912de048102c96f7c24df5dfc" style="zoom: 67%;" />

2. 回到工作空间根目录编译：

   ```bash
   cd ..
   catkin_make
   ```

3. 开启 ROS：

   ```bash
   roscore
   ```

4. 执行 data_convert_node 数据转换节点，稍作等待：

   ```bash
   source /home/lzx/code/kf-gins-ros-ws/devel/setup.bash	# 换成自己的目录
   rosrun data_convert data_convert_node
   ```

   ![aa55dee2f3ee139d2277a3bfb01a19ed](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/aa55dee2f3ee139d2277a3bfb01a19ed.png)

   转换出 ROSBag 之后我们可以查看其信息：

   ```bash
   rosbag info /home/lzx/code/kf-gins-ros-ws/src/kf-gins-ros/data2bag/output.bag # 换成自己的路径
   ```

   ![4176ebc43de2323e6121a55bd37667b0](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/4176ebc43de2323e6121a55bd37667b0.png)

5. 开启 gins_node 节点，它会等待 imu/gnss 数据

   ```bash
   rosrun gins gins_node /home/lzx/code/kf-gins-ros-ws/src/kf-gins-ros/kf_gins_ros/config/gins.yaml # 路径换成自己的
   ```

   在另一个终端中播放 rosbag，多等一会，gins_node 就会开始输出解算信息：

   ```bash
   rosbag play /home/lzx/code/kf-gins-ros-ws/src/kf-gins-ros/data2bag/output.bag # 路径换成自己的
   ```

   ![8382907fe51c86cede00569976e41fa5](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/8382907fe51c86cede00569976e41fa5.png)

6. 

#### 3. 在 VSCode 中编译调试





![7a58a41815b7c7e64235760db6e46fec](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7a58a41815b7c7e64235760db6e46fec.png)

经过调试可以看出每组数据的解算流程：

![8958bea17192bafc4337bf8706250b39](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/8958bea17192bafc4337bf8706250b39.png)





### 4、以 roscpp_tutorials 为例学习 ROS 项目创建及发布者、订阅者程序的编写

`roscpp_tutorials` 中的 **talker** 和 **listener** 是两个经典的示例程序，用于演示 ROS 中最基本的通信机制——**话题（Topic）**。`talker` 节点是一个发布者（Publisher），它会以固定频率向名为 `/chatter` 的话题发布字符串消息；通过 `ros::Publisher` 创建发布者对象，并使用 `publish()` 方法发送 `std_msgs::String` 类型的消息。`listener` 节点则是一个订阅者（Subscriber），它订阅 `/chatter` 话题，并通过回调函数接收和处理来自 `talker` 的消息；每当有新消息时，回调函数会被触发，并将消息内容打印到终端。这两个示例展示了 ROS 中发布-订阅模式的基本用法，是学习 ROS C++ 编程的重要起点。

#### 1. 创建空间和功能包

1. `mkdir` 命令创建工作空间：

   ```bash
   mkdir -p ros_test/src
   cd ros_test/
   ```

2. 初始化工作空间，在目录下生成一个 `CMakeLists.txt` 文件（似乎执行不执行这句都可以）：

   ```bash
   catkin_init_workspace
   ```

3. `catkin_make` 编译：

   ```bash
   catkin_make
   ```

4. 在工作空间下打开 VSCode：

   ```bash
   code .
   ```

3. 执行 `source` 命令将启动文件加到环境变量

   ```bash
   source devel/setup.bash
   ```

4. 在 `src` 目录下用 `catkin_create_pkg` 命令创建功能包：

   ```bash
   cd src
   catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
   ```

#### 2. 加入 talker.cpp、listener.cpp 源文件

1. 在 `src/beginner_tutorials/src/` 创建 `talker.cpp` 文件夹，并将下面代码复制进去：

   ```c++
   #include "ros/ros.h"            // ROS C++ 客户端库的核心头文件，包含了 ROS 的基本功能（如节点初始化、话题通信等）
   #include "std_msgs/String.h"    // ROS 标准消息类型之一，表示字符串消息。
   #include <sstream>              // C++ 标准库中的字符串流头文件，用于构建字符串。
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "talker");  // 初始化 ROS 节点
     ros::NodeHandle n;                // 创建节点句柄，用于与 ROS 系统通信（如创建订阅者、发布者等）。
     
     // 创建一个发布者对象，参数包括：
     //  - chatter：要订阅的话题名称。
     //  - 1000：消息队列长度，表示最多缓存 1000 条未处理的消息。
     //  - <std_msgs::String>：消息类型，这里是字符串消息。
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
     ros::Rate loop_rate(10);  // 设置循环频率为 10 Hz，即每秒循环 10 次
     
     // 主循环，只要 ROS 节点正常运行（未被关闭），就会一直执行。
     // ros::ok() 会检查节点的状态（如是否收到终止信号）。
     int count = 0;
     while (ros::ok())     
     {
       // 构建消息内容
       std_msgs::String msg;
       std::stringstream ss;
       ss << "hello world " << count;
       msg.data = ss.str();
   
       // 使用 ROS_INFO 打印消息内容到终端。
       ROS_INFO("%s", msg.data.c_str());
   
       // 将消息 msg 发布到 chatter 话题。
       chatter_pub.publish(msg);
   
       // 处理一次回调函数（如果有）。
       // 对于发布者节点，通常不需要处理回调，但保留 ros::spinOnce() 是一个好习惯。
       ros::spinOnce();
   
       // 根据设置的频率（10 Hz）休眠一段时间，确保循环每秒执行 10 次。
       loop_rate.sleep();
   
       // 计数器 count 自增，用于生成不同的消息内容。
       ++count;
     }
   
     return 0;
   }
   ```

2. 在 `src/beginner_tutorials/src/` 创建 `listener.cpp` 文件夹，并将下面代码复制进去：

   ```c++
   #include "ros/ros.h"            // ROS C++ 客户端库的核心头文件，包含了 ROS 的基本功能（如节点初始化、话题通信等）
   #include "std_msgs/String.h"    // ROS 标准消息类型之一，表示字符串消息。
   
   // 当订阅的话题有新消息时，ROS 会自动调用此函数，使用 ROS_INFO 打印消息内容
   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
     ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "listener");    // 初始化 ROS 节点
     ros::NodeHandle n;                    // 创建节点句柄，用于与 ROS 系统通信（如创建订阅者、发布者等）。
   
     // 创建一个订阅者对象，参数包括：
     //  - chatter：要订阅的话题名称。
     //  - 1000：消息队列长度，表示最多缓存 1000 条未处理的消息。
     //  - chatterCallback：回调函数，用于处理接收到的消息。
     ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
     
     // 进入 ROS 事件循环，等待并处理消息。
     // 当有新消息到达时，ROS 会自动调用注册的回调函数（如 chatterCallback）。
     ros::spin();
     
     return 0;
   }
   ```

3. 在 `src/beginner_tutorials/CMakeLists.txt` 文件中加入以下语句，将 `talker.cpp` 编译成 `talker_node`，将 `listener.cpp` 编译成 `listener_node`。 

   ```cmake
   add_executable(talker_node src/talker.cpp)   
   target_link_libraries(talker_node ${catkin_LIBRARIES}) 
   
   add_executable(listener_node src/listener.cpp)               
   target_link_libraries(listener_node ${catkin_LIBRARIES})  
   ```

#### 3. 编译运行 talker_node、listener_node

1. 先 `Ctrl + Shift + B` 编译：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/adcf879c6e8e18467eb670b833390189.png" alt="adcf879c6e8e18467eb670b833390189" style="zoom:67%;" />，

   > ROS 项目 catkin_make 编译的过程中如果报错“The specified base path "XXX" contains a CMakeLists.txt but "catkin_make" must be invoked in the root of workspace"，可以在工作空间根目录执行命令：
   >
   > ```bash
   > unlink ./CMakeLists.txt
   > ```

2. 在三个终端中分别输入

   ```bash
   roscore
   ```

   ```bash
   source code/ros_test/devel/setup.bash   # 改成自己的目录
   rosrun beginner_tutorials listener_node # 启动 listener_node
   ```

   ```bash
   source code/ros_test/devel/setup.bash   # 改成自己的目录
   rosrun beginner_tutorials talker_node 	# 启动 talker_node
   ```

   可以看出 talker_node 不断发送 “hello world xxx”，listener_node 接收 talker_node  消息并打印到终端。我们可以用 rqt_graph 来查看系统图：

   ![a3da68c59f2d2c6083327e53e9045374](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/a3da68c59f2d2c6083327e53e9045374.png)

#### 4. 配置 launch.json 文件，用 VSCode 调试

1. 调试之前，先 `Ctrl + Shift + P` 选择 ROS Start 启动（相当于 `roscore`）：

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/166164a0fd4a22dd7b3059782c6fa4fb.png" alt="166164a0fd4a22dd7b3059782c6fa4fb" style="zoom: 80%;" />

2. 创建 launch.json 文件，创建 C++ 调试任务，程序路径改为 devel 目录下的 talker_node：

   ```bash
   ${workspaceFolder}/devel/lib/beginner_tutorials/talker_node
   ```

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/4b84b8c9c26c4b2d8a00620ad904bcb0.png" alt="4b84b8c9c26c4b2d8a00620ad904bcb0" style="zoom:50%;" />

### 5、话题消息的定义和使用











---

## 附录一：ROS常用命令

![ROS_Cheat_Sheet_Hydro-逐页转图片](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/ROS_Cheat_Sheet_Hydro-%E9%80%90%E9%A1%B5%E8%BD%AC%E5%9B%BE%E7%89%87.jpg)

### 基本操作

|                操作                |                             命令                             |
| :--------------------------------: | :----------------------------------------------------------: |
|           启动节点管理器           |                           roscore                            |
|         一次性启动多个节点         |                          roslaunch                           |
|              启动节点              |           rosrun [package-name] [executable-name]            |
|          获得运行节点列表          |                         rosnode list                         |
| 使用 rosrun 命令显式设置节点的名称 | rosrun [package-name] [executable-name] __name:=[node-name]  |
|        安装 turtlesim 功能         |            sudo apt install ros-kinetic-turtlesim            |
|           启动 turtlesim           | roscore、rosrun turtlesim turtlesim_node、rosrun turtlesim turtle_teleop_key |
|         在功能包内启动文件         |             roslaunch package [filename.launch]              |
|         在局部节点启动文件         |          roslaunch –local package [filename.launch]          |
|       安装一个包的系统依赖项       |                   rosdep [filename.launch]                   |
|              编辑文件              |                            rosed                             |
|            加载参数文件            |                      rosparam load file                      |
|           输出参数到文件           | rosparam dump file、rosparam delete parameter、rosparam list |

### 功能包操作

|             操作             |                命令                 |
| :--------------------------: | :---------------------------------: |
|  查看软件包列表和定位软件包  |            rospack list             |
|     找到一个软件包的目录     |     rospack find [package-name]     |
|        查看包的依赖项        | rsopack depends tur turtlesim_node  |
|    查看软件包目录下的文件    |        rosls [package-name]         |
| 将当前目录切换到此软件包目录 |        roscd [package-name]         |
|      在功能包内启动文件      | roslaunch package [filename.launch] |
|    安装一个包的系统依赖项    |      rosdep [filename.launch]       |
| 以图形界面显示一个包的依赖项 |               rqt_dep               |
|      从一个包中复制文件      |                roscp                |
|       列出一个包的目录       |                rosd                 |

### 节点操作

|           操作           |                         命令                         |
| :----------------------: | :--------------------------------------------------: |
| 查看所有正在运行的 Node  |                     rosnode list                     |
|        运行 Node         | rosrun [package_name] [node_name] [__name:=new_name] |
|       查看节点信息       |               rosnode info [node-name]               |
|         终止节点         |               rosnode kill [node-name]               |
|    将节点从列表中删除    |                   rosnode cleanup                    |
|  查看节点之间的连接关系  |                      rqt_graph                       |
|  显示关于节点的调用信息  |                    rosnode [xxx]                     |
| 测试到一个节点的可连接性 |                  rosnode ping [xxx]                  |
|     列出所有活动节点     |                  rosnode list [xxx]                  |
|       列出节点信息       |                  rosnod info [xxx]                   |
|      结束运行的节点      |                  rosnod kill [xxx]                   |
|       结束所有节点       |                   rosnode kill -a                    |

### 话题操作

|              操作              |                             命令                             |
| :----------------------------: | :----------------------------------------------------------: |
|       列出当前活跃的话题       |                        rostopic list                         |
|    查看某个话题上发布的消息    |                   rostopic echo topic-name                   |
|        查看话题消息格式        |        rostopic type [topic]、rosmsg show [msg_type]         |
|     输出每秒发布的消息数量     |                   rostopic hz [topic-name]                   |
|  输出每秒发布消息所占的字节量  |                   rostopic bw [topic-name]                   |
|     获取更多关于话题的信息     |                  rostopic info [topic-name]                  |
|         查找特定的节点         |                 rostopic list \| grep [XXX]                  |
|          查看消息类型          |               rosmsg show [message-type-name]                |
|          发布制定消息          | rostopic pub –r rate-in-hz [topic-name] [message-type] [message-content] |
| 显示消息或者服务的数据结构定义 |                        rosmsg/rossrv                         |
|       显示消息中域的定义       |                         rosmsg show                          |
|     显示调用指定消息的代码     |                         rosmsg users                         |
|  列出指定功能包中的所有的消息  |                        rosmsg package                        |
|   列出带有该消息的所有功能包   |                       rosnode packages                       |

### 编译操作

|                  操作                  |                            命令                             |
| :------------------------------------: | :---------------------------------------------------------: |
|              编译指定的包              | catkin_make -DCATKIN_WHITELIST_PACKAGES=”package1;package2″ |
|            恢复编译所有的包            |         catkin_make -DCATKIN_WHITELIST_PACKAGES=””          |
|       提取⽂件系统上的功能包信息       |                           rospack                           |
| 用于⽂件系统上的功能包集信息的命令⼯具 |                          rosstack                           |
|           查找到某个功能包集           |                     rosstack find [xxx]                     |

### 图形界面操作

|              操作              |                      命令                      |
| :----------------------------: | :--------------------------------------------: |
| 以界面的形式显示正在运行的节点 |             rosrun rqt_top rqt_top             |
|  界面的形式显示话题的调试信息  |                   rqt_topic                    |
|   界面的形式显示订阅者的信息   |                 rqt_publisher                  |
|     以图形界面调用服务信息     |               rqt_service_calle                |
|       查看节点发出的消息       |                  rqt_console                   |
|          设置动态参数          |                rqt_reconfigure                 |
|            节点监测            | rosrun rqt_runtime_monitor rqt_runtime_monitor |
|        监测所有节点信息        |   rosrun rqt_robot_monitor rqt_robot_monitor   |
|      绘制某个消息的曲线图      |         rosrun rqt_plot rqt_plot [xxx]         |
|        显示三个消息参数        |    rosrun rqt_plot rqt_plot [/accel/x:y:z]     |
|            查看图片            |      rosrun rqt_image_view rqt_image_view      |
|   以图形界面查看包的内部信息   |                    rqt_bag                     |

### Bag 操作

|        操作         |                   命令                    |
| :-----------------: | :---------------------------------------: |
| 记录所有 topic 变化 |             rosbag record -a              |
|   记录某些 topic    | rosbag record -O subset [topic1] [topic2] |
|    查看 bag 信息    |        rosbag info [bagfile_name]         |
|        回放         |     rosbag play (-r 2) [bagfile_name]     |

## 附录二：相关网址

* ROS官网：https://wiki.ros.org/
* Ubuntu官网下载：https://ubuntu.com/download
* VSCode插件市场：https://marketplace.visualstudio.com/VSCode
* KF-GINS：https://github.com/i2Nav-WHU/KF-GINS
* kf-gins-ros：https://github.com/slender1031/kf-gins-ros
* wpr_simulation：_https://github.com/6-robot/wpr_simulation
* 机器人操作系统 ROS 快速入门教程-张万杰：https://www.bilibili.com/video/BV1BP4y1o7pw
* ROS入门21讲-古月：https://www.bilibili.com/video/BV1zt411G7Vn
* ROS命令一页纸：www.clearpathrobotics.com/ros-cheat-sheet
* 搜狗输入法 for Linux 安装指南：https://pinyin.sogou.com/linux/help.php

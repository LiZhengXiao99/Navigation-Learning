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
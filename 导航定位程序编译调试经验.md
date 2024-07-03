<div align="center">
    <a name="Top"></a>
	<h1>Navigation-Debug：导航定位程序编译调试经验</h1>
    <p><strong>记录一些导航程序编译调试过程中遇到的问题，和我找到的解决方案，以后遇到啥问题了，都来记录一下</strong></p>
    <p><strong>如果针对我提出的问题，您有更好的解决方案，欢迎提 Issue 分享！</strong></p>
</div>
<div align="center">
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
</div>

---

### 001-单步调试 C++ 程序总是进入标准库函数

> 解决办法有两种：
>
> * 遇到标准库函数的语句，用单步跳过（逐过程），而不是单步执行，已经进入库函数了就单步跳出；
>
> * 改编译器配置，一劳永逸的解决问题
>
>   * **VS**：改 default.natstepfilter 文件：
>
>     ```bash
>     C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Packages\Debugger\Visualizers\default.natstepfilter
>     ```
>
>     以管理员身份打开，并在 xml 格式中加入一行：
>
>     ```bash
>     <Function><Name>std::.*</Name><Action>NoStepInto</Action></Function>
>     ```
>
>   * **VSCode**：改每个项目的 Lunch.json 文件，在 "setupCommands" 里加上：
>
>     ```bash
>     {
>     	"description": "防止gdb打开标准库函数",
>     	"text": "-interpreter-exec console \"skip -rfu std::.*\"",
>     	"ignoreFailures": false
>     }
>     ```
>
>     > 参见：[避免vscode调试C++时进入标准库 | VScode调试C++不进入标准库的配置方法](https://zhuanlan.zhihu.com/p/576544599)

---

### 002-Windows下编译链接的时候找不到 XXX 文件

> * 出现这个问题的原因有很多，可能是你确实没有这个文件，可能是没放在环境变量或者默认包含路径里，可能是头文件写的包含路径不对；
>
> * 还有一个可能的原因是因为没开管理员权限，我用 Arduino、Qt Creater、Keil 的时候都遇到过这种问题；可以右击软件图表，选择【属性】，在【兼容性】选项卡中，勾选【以管理员身份运行此程序】；设置成功之后，软件图标右下角应该会显示一个盾牌，之后每次打开直接就有管理员权限。

---

### 003-Qt 在线安装下载速度太慢

> * 如果是 Qt 5.15 以前的版本，可以下载离线安装包：
> 	* 中国科学技术大学：http://mirrors.ustc.edu.cn/qtproject/
> 	* 清华大学：https://mirrors.tuna.tsinghua.edu.cn/qt/
> 	* 北京理工大学：http://mirror.bit.edu.cn/qtproject/
> 	* 中国互联网络信息中心：http://mirror.bit.edu.cn/qtproject/
>
> * 针对 Qt 安装器下载慢：**去镜像网站下载**：http://mirrors.ustc.edu.cn/qtproject/official_releases/online_installers/
>
> * 针对 Qt 库在线安装时下载慢：**切换到镜像源**：网上有些文章说用 flddler 改 HTTP 代理地址，完全是多此一举，根本要不了那么麻烦。新版本的安装器（4.0.1-1 后）本身就提供了切换镜像的功能。在命令行中执行安装器，添加 `--mirror https://mirrors.ustc.edu.cn/qtproject `参数。例如 Windows 下执行当前目录的安装器的命令为：
>
>   ```bash
>   ./qt-unified-windows-x86-online.exe --mirror https://mirrors.ustc.edu.cn/qtproject
>   ```
>
>   你可以先在 Qt 安装器的目录，打开 cmd，输入`./<Qt安装器文件名>`，如果能打开 Qt 安装，则在后面再加上 `--mirror https://mirrors.ustc.edu.cn/qtproject `，进行安装。

---

### 004-Windows 下打开（尤其是C++写的）软件显示缺少 “xxx.dll” 文件

> * “dll” 是Dynamic Link Library（动态链接库）的缩写，静态库在编译链接的时候就要引入，而动态库在程序执行的时候才引入，常用于存储可由多个程序共享的函数或数据；
> * 打开程序报错缺少的 “xxx.dll” 文件一般是系统自带或者常用程序的动态链接库，可以去 dll 网站（比如：[找dll](https://www.zhaodll.com/)）下载缺失的 .dll 文件，放到 `C:\Windows\SysWOW64` 或 `C:\Windows\System32` 目录下面。
> * 有些 DLL 文件看名字就能猜出它是啥，比如 `Qt5Help.dll`，肯定是 Qt 里的，咱们需要把 `<Qt安装目录>\mingw\bin` 添加到系统环境变量。

---

### 005-Matlab 打开别人的程序，中文注释乱码

> * 这是编码格式的问题，可能别人的程序是 GBK 而我们的是 UTF-8，可以参考 [Matlab不同版本之间的乱码问题](https://zhuanlan.zhihu.com/p/590985353) 来修改。

---

### 006-Matlab 图表中显示中文字符为 □

> * 新建一个 startup.m 文件，放到你安装位置，然后在 .m 文件里面 set() 有中文的字体。
> * 可以参考：[matlab编程或者画图时输出汉字为□□？](https://zhuanlan.zhihu.com/p/678752167)

---

### 007-调试 C/C++ 程序出现异常，直接进汇编代码，不知道是哪里的问题

> * 大部分编译器都可以显示调用堆栈，可以让我们直接看到执行哪一行代码出了问题；
> * 最常见的异常是访问没初始化的变量和字段、访问已经释放的指针；

---

### 008-使用 Eigen 库编译时报一堆错，不知道是哪里的问题

> * Eigen 是 C++ 的一个开源线性代数库，主要是进行矩阵运算，对导航算法相当关键，要知道导航算法基本都是矩阵计算。除了基本的矩阵计算之外，支持四元数、旋转矩阵，C++ 写的导航定位的开源代码中基本都会用到 Eigen。
> * 使用 Eigen 有个麻烦的地方就在于难以调试，经常报错了，你找不到问题在哪，尤其是一口气写了一大段。有的错误写出来的时候编译器就会提示你错了，有的编译的时候就报错并且告诉你哪一行出错了，这两种都还比较友好；麻烦的是的是有的编译的时候报错不告诉你哪一行错，有的编译能过，运行到那一行才出错。
> * 我习惯写两行就编译一下，写几句就单步调试一下，要不然写多了不好调试。

---

### 009-使用 VSCode 调试 C/C++ 程序看不到指针指向变量的值

> * 在监视窗口输入要查看表达式来查看；
> * 数组可能还是不方便直接看，得输出到终端或者文件才能看到：
>   * RTKLIB 及其二次开发程序可以用 tracemat() 函数；
>   * Eigen 库重载了 <<，可以直接 Cout 输出到数据流；

---

### 010-Linux 下编译 Cmake 构建的程序，找不到已经安装过的库

> * 可能是因为库装到`/usr/local/lib` 里了，试试创建软链接到 `/usr/lib`：
>
>   ```bash
>   ln -s /usr/local/lib/库名.a /usr/lib/库名.a
>   ```
>   
> * 也可以直接把 `/usr/local/lib` 也添加到包含目录。

### 011-VS 下设断点显示“当前不会命中断点。还没有为该文档加载任何符号。”

> 可能是设置成了 Relese 模式，改成 Debug 模式才能断点调试。

---

### 012-Windows 下用 VSCode 编译 KF-GINS 无法进入断点

> 把 lunch.json 文件中的 `"type": "cppvsdbg"` 改为 `"type": "cppdbg"`

---

### 013-WSL 通过 VcXsrv 使用 xfce4 图形界面报错：“Can‘t open display”、“Connection refused”

> * `vim ~./bashrc` 在末尾加上 `export DISPLAY=127.0.0.1:0`，或者后面填自己的 IP 地址，保存退出。
> * 执行命令： `source ~/.bashrc`；
> * 打开 Xluanch ，前两步默认设置就行，第三步中 Disable access control 一定要选上；
> * 执行命令：`startxfce4`，即可启动界面。

---

### 014-.sh shell 脚本无法执行，提示 “command not found”、“permission denied”

> * 提示 permission denied 显然是因为要管理员权限，可以在命令前面加上 `sudo`；
> * 提示 command not found 可能是因为文件没有执行权限，可以通过命令来添加：`chmod +x <.sh 脚本名>`；

---

### 015-VSCode 远程安装插件特别慢

> 要配置 VSCode + WSL 的开发环境，需要在 WSL 里也装上 C++、CMake 系列的插件，直接下载方式特别慢。
>
> * 先去网站下载后缀名为 .VXIS 的插件：https://marketplace.visualstudio.com/；
> * 把 VXIS 文件放到 WSL 中，VScode 中可以直接把资源管理器中的文件托入左侧文件列表；
> * VSCode 中选择通过 VXIS 安装，在下来菜单中选择要安装的 VXIS 文件，稍等片刻即安装成功。

---

### 016-远程控制 ROS 没有访问权限

> * 远程控制 ROS 需要从机和主机在同一个局域网下面，也就是连同一个 WiFi 或者热点，或者一个电脑机连一个电脑的热点；
>
> * 需要改 .bashrc 文件设置多机通信的环境：
>
>   * 在 ROS 主机的 .bashrc 文件最后添加：
>
>     ```bash
>     export ROS_MASTER_URI=http://(这里填ros主机的IP地址):11311
>     export ROS_HOSTNAME=(这里填ros主机的IP地址)
>     ```
>
>   * 在 ROS 从机的 .bashrc 文件最后添加：
>
>     ```bash
>     export ROS_MASTER_URI=http://(这里填ros主机的IP地址):11311
>     export ROS_HOSTNAME=(这里填ros从机的IP地址)   # ifconfig查询
>     ```

---

### 017-Keil 下载程序到单片机报错“找不到仿真器”、下载完程序不自动运行

> * 如果你缺少连了仿真器，但还是说找不到，可能是因为项目设置中仿真器没选择好，进入【Option for Target】，在【Debug】选项卡的右上角选择对应的仿真器；
> * 默认下载程序之后都不会自动运行，按一下单片机上的复位按键手动复位之后，就会执行新烧录的程序；
> * 如果你希望程序烧录之后，单片机立刻复位执行，可以进入【Option for Target】，点【Debug】选项卡右上角的【Setting】进入程序下载配置窗口，把【Flash DownLoad】选项卡的【Reset and Run】勾选上。

---

### 018-Keil 中文注释乱码、串口输出中文乱码

> * 中文注释乱码：打开【Configuration🔧】在【Editor】选项卡中设置【Encoding】为UTF-8；
> * 串口输出中文乱码：打开【Option for Target】，在【C/C++】选项卡中将【MiscControls】填写为：`--no-multibyte-chars`。

---

### 019-把文件名作为命令行参数时会遇到的问题

> * Linux 文件路一般是 `/usr/bin`，很少出问题；
> * Window 文件路径一般是 `C:\windows\system32`，比较容易出问题，因为 `\` 是 C/C++ 中的转义字符，会将后面的字符转义，不能直接输入，需要把目录中的 `\` 换成 `/` 或者 `//`；
> * 在 VS 中的项目配置输入命令行参数时，直接输原本的路径就好，VS 会自动将输入参数的 `/` 改为 `//`；

---

### 020-VSCode 调试程序的时候，程序不执行，出现“因ENTRY已暂停”

> * 这是因为在 lunch.json 文件中设置了 `"stopAtEntry": true`，在程序入口处暂停；

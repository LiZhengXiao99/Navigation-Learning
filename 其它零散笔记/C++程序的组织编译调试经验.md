[TOC]



对咱们来说，所有的理论都是用代码实现了才有意义，脱离代码学习理论毫无作用，如果不只是为了应付学校的课程，想真的学会点什么，必须要看代码写代码；编译调试是学习开源程序的最重要的一步，会编译你才能去尝试改代码，会调试你才能去深入的分析代码的执行情况。



对于导航定位算法研究者和开发者来说，编程主要就是处理数据，

* **数据获取**：可以是文本文件、二进制文件、可以是串口、网络的数据流、也可以是 rosbag 等形式。
* **数据处理**：实现导航定位的算法，矩阵运算肯定是绕不开的，常用 numpy、Eigen、OpenBLAS 库。
* **结果输出**：





包括：

* **开发环境**：VS、VSCode、Clion、Qt Creater、Keil、Arduino IDE
* **程序组织**：VS、GCC、Make、Cmake、Qmake、ROS1&2
* **单片机**：51、Arduino、ESP32、STM32、ZYNQ
* **相关工具**：Git、Docker



## 一、C/C++介绍

在正文的最开始，首先来介绍一下 C、C++ 发展

### 1、C语言



C 最初用于重新实现 Unix 操作系统，该操作系统是从汇编语言用C重写的，它因其可移植性和高效性而迅速流行起来。因此，Unix操作系统的开发始于1969年。1972年，它的代码被用C编程语言重写。

作为一门1970年代发明出来的语言，那个年代甚至连一 Byte 有多少 bit 都没定下来，C 语言的出现对当时的计算机编程觉得是革命性的。

C 语言允许直接访问物理地址，可以直接对硬件进行操作，既具有高级语言的功能，又具有低级语言的许多功能，能够像汇编语言一样对位、字节和地址进行操作，而这三者是计算机最基本的工作单元，可以用来写系统软件、驱动、单片机程序。

C语言自诞生到现在，期间经历了多次标准化过程，主要分成以下几个阶段



|            |                                                              |
| :--------: | ------------------------------------------------------------ |
| **“K&R”C** | 也称为 Traditional-C，当时的 C 语言还没有标准化，来自 “C Programming Language, First Edition”  的 C 语言描述可算作“正式”的标准，期间 C 语言一直不断的发生细微的变化，各编译器厂商也有自己的扩展，这个过程一直持续到20世纪80年代末。 |
|  **C89**   | 也称为 ANSI C，考虑到标准化的重要，ANSI（American National Standards Institute）制定了第一个 C 标准，在1989年被正式采用。在此标准中定义了 C 标准库、新的预处理命令和特性、加入了关键字（const、volatile、signed） |
|  **C95**   | 也称为 “C89 with Amendment 1”，是对 C89 的一个修订和扩充     |
|  **C99**   | 应该是目前使用最广泛的 C 语言标准，加入了很多重要的特性，包括：Boolean 类型、非英语字符集支持、提供全部类型的数学函数、C++ 风格注释（`//`）等 |
|  **C11**   | 添加了多线程和原子操作等                                     |
|  **C17**   | 对内存安全、多线程等进行了改进                               |
|  **C23**   |                                                              |

除此以为还用适用于嵌入式的



### 2、C++



C++ 是在C语言的基础上发展而来

兼容绝大多数 C语言的语法，

会写 C++ 的一定会写 C，虽然不一定写的好，

建议学完 C 再学 C++，



Boost 库，在 C++11 之前被广泛的使用，包括 gnss-sdr，

其中比较实用的特性诸如智能指针、互斥量等都与 C++11 类似，按 C++11 里的理解就可以了，不用专门去学 Boost 库，也不建议去用 Boost 库。





C++先后有



C++11 之后，保持着 3 年一个的



截止到 24 年初，笔者写这篇文档的时间，GCC 已经支持了 C++20 之前全部的语法、C++23 的绝大多数语法、C++ 26 的大部分语法。



### 3、C/C++ 学习经验







不推荐之类



推荐三本学习 C++ 对我最有帮助的书：

* **《C++ Premier》**：直译过来叫“C++入门”，但不适合完全零基础看，难度比其它的 C++ 入门书籍、教材大不少，写的很详细，文笔还非常幽默风趣让人心情愉悦，还有个简略的版本 **《C++ Premier Plus》**。
* **《Effective C++》**：，使用的 C++ 版本比较老，很多内容不适用于现代 C++，
* **《C++20高级编程》**：

| ![image-20240302165218673](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240302165218673.png) | ![image-20240302165356230](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240302165356230.png) | ![image-20240302165409048](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240302165409048.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |



推荐一个 C++ 网站：Hacking C++，

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240302161806768.png" alt="image-20240302161806768" style="zoom: 50%;" />

系统形象，简单直接，比看大段大段的文字描述强太多了











## 二、VS + MSVC

VS 是很多人入门 C/C++ 最早使用的开发环境，对于入门来说，它足够的简单，一键编译调试，点点配置就能组织好一个程序，引入第三方库，加入环境变量，都很方便。











## 三、GCC、Make、GDB、Cmake



C/C++有三大编译器：GCC、MSVC、Clang，其中 GCC 使用最为广泛，是 GNU/Linux 钦定的编译器，MSVC 一般与 VS 绑定。



传统的编译器通常分为三个部分，前端（frontEnd），优化器（Optimizer）和后端（backEnd）. 在编译过程中，前端主要负责词法和语法分析，将源代码转化为抽象语法树；优化器则是在前端的基础上，对得到的中间代码进行优化，使代码更加高效；后端则是将已经优化的中间代码转化为针对各自平台的机器代码。



GCC（GNU Compiler Collection，GNU 编译器套装），是一套由 GNU 开发的编程语言编译器。GCC 原名为 GNU C 语言编译器，因为它原本只能处理 C语言。GCC 快速演进，变得可处理 C++、Fortran、Pascal、Objective-C、Java 以及 Ada 等他语言。



GCC 的编译过程可以划分为四个阶段：

* **预处理（Pre-Processing）**：
* **编译（Compiling）**：
* **汇编（Assembling）**：
* **链接（Linking）**：





|         后缀         |          描述           |     后缀     |      描述      |
| :------------------: | :---------------------: | :----------: | :------------: |
|        **.c**        |        C 源文件         |  **.s/.S**   | 汇编语言源文件 |
| **.C/.cc/.cxx/.cpp** |       C++ 源文件        | **.o/.obj**  |    目标文件    |
|     **.h/.hpp**      |      C/C++ 头文件       | **.a/.lib**  |     静态库     |
|      **.i/.ii**      | 经过预处理的 C/C++ 文件 | **.so/.dll** |     动态库     |

程序员可以根据自己的需要控制 GCC 的编译阶段，以便检查或使用编译器在该阶段的输出信息，帮助调试和优化程序。





make工具可以看成是一个智能的批处理工具，它本身并没有编译和链接的功能，而是用类似于[批处理](https://www.zhihu.com/search?q=批处理&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A"638986464"})的方式—通过调用makefile文件中用户指定的命令来进行编译和链接的。makefile是什么？makefile就是一个[脚本文件](https://www.zhihu.com/search?q=脚本文件&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A"638986464"})，简单的说就像一首歌的乐谱，make工具就像[指挥家](https://www.zhihu.com/search?q=指挥家&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A"638986464"})，指挥家根据乐谱指挥整个乐团怎么样演奏，make工具就根据makefile中的命令进行编译和链接的。makefile命令中就包含了调用gcc（也可以是别的编译器）去编译某个源文件的命令。





MinGW，是 Minimalist GNU for Windows 的缩写。它是一个可[自由使用](https://baike.baidu.com/item/自由使用/60563740?fromModule=lemma_inlink)和自由发布的Windows特定[头文件](https://baike.baidu.com/item/头文件/10978258?fromModule=lemma_inlink)和使用GNU[工具集](https://baike.baidu.com/item/工具集/5123149?fromModule=lemma_inlink)[导入库](https://baike.baidu.com/item/导入库/10051541?fromModule=lemma_inlink)的集合，允许你在[GNU/Linux](https://baike.baidu.com/item/GNU/Linux/7061928?fromModule=lemma_inlink)和Windows平台生成本地的[Windows程序](https://baike.baidu.com/item/Windows程序/15644576?fromModule=lemma_inlink)而不需要第三方C[运行时](https://baike.baidu.com/item/运行时/3335184?fromModule=lemma_inlink)（[C Runtime](https://baike.baidu.com/item/C Runtime/10660179?fromModule=lemma_inlink)）库。MinGW 是一组包含文件和端口库，其功能是允许控制台模式的程序使用[微软](https://baike.baidu.com/item/微软/124767?fromModule=lemma_inlink)的标准C运行时（C Runtime）库（[MSVCRT.DLL](https://baike.baidu.com/item/MSVCRT.DLL/0?fromModule=lemma_inlink)）,该库在所有的 NT OS 上有效，在所有的 [Windows 95](https://baike.baidu.com/item/Windows 95/0?fromModule=lemma_inlink)发行版以上的 Windows OS 有效，使用基本运行时，你可以使用 GCC 写控制台模式的符合美国[标准化组织](https://baike.baidu.com/item/标准化组织/873654?fromModule=lemma_inlink)（[ANSI](https://baike.baidu.com/item/ANSI/14955?fromModule=lemma_inlink)）程序，可以使用微软提供的 C 运行时（C Runtime）扩展，与基本运行时相结合，就可以有充分的权利既使用 CRT（C Runtime）又使用 WindowsAPI功能。





makefile在一些简单的工程下，完全可以人工手写，但是当工程非常大的时候，手写makefile也是非常麻烦。而且陆陆续续出现了各种不同平台的makefile，有GNU make、QT 的 [qmake](https://www.zhihu.com/search?q=qmake&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A"638986464"})，微软的 MSnmake，BSD Make（pmake），Makepp等等。这些 Make 工具遵循着不同的规范和标准，所执行的 Makefile 格式也千差万别。这样就带来了一个严峻的问题：如果软件想跨平台，必须要保证能够在不同平台编译。而如果使用上面的 Make 工具，就得为每一种标准写一次 Makefile ，这将是一件让人抓狂的工作，如果换了个平台makefile又要重新修改。





CMake 就是针对上面问题所设计的工具：它首先允许开发者编写一种平台无关的 CMakeList.txt 文件来定制整个编译流程，然后再根据目标用户的平台进一步生成所需的本地化 Makefile 和工程文件，从而做到”一次编写，到处运行“。

笔者接触过得到绝大多数导航定位程序的都用 Cmake 来组织，包括 TGINS、GICI、GNSS-SDR、Ginan 等。



很多常用的 C++ 库如 Eigen、Ceres、OpenBLAS、OpenCV 也都用 CMake，想要用这些库，一般都是先要 Git 下载源码，然后 Cmake、Make 编译。



基于 Cmake 构建的项目你可以完完整整的了解到 C++ 程序的组织方式，



CLion 只支持 Cmake 程序，ROS 程序的组织方式都是基于 Cmake





## 四、VSCode 和 CLion 编译调试 CMake 组织的 C++ 程序





## 五、Qt 

Qt 是 C/C++ 重要的开发框架，

可以用于开发界面程序，包括桌面应用、嵌入式、工控上位机

跨平台

Qmake类似于CMake的东西，是用来生成makefile的





## 六、ROS



ROS 全称（）机器人操作系统，主要运行在 Linux 上

绝大多数的 SLAM 程序都基于 ROS

包括一套通信框架和众多的功能包

一些简单的机器人比如扫地机、



因为性能、实时性等原因很多成熟的SLAM设备、机器人、自动驾驶系统并不会使用直接 ROS，但是在产品开发的早期还会 ROS，

自动驾驶框架 AutoWare、Apollo 也都在很大程度上模仿借鉴了 ROS，掌握 ROS 开发基本是学习它们的前置条件







## 七、STM32、Arduino、ESP32





STM32 可以用的开发工具很多，

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240302180914334.png" alt="image-20240302180914334" style="zoom: 50%;" />

最常用的包括：

* **Keil**：最经典的嵌入式MCU开发环境，无论是 51 还是ARM Cortex M0-7，KEIL都有着不错的支持。多数人在开发`STM32`的时候用的都是`Keil`这个老牌IDE，很大一部分原因是因为大多数人最初是从51单片机学习过来的，51就是基于`Keil`去开发的，然后迁移到`STM32`的时候也就沿用下来了。
* **STM32CubeIDE**：ST意法半导体自己出品，用来开发自家MCU产品，体验良好。其本质是集成了 STM32CubeMX 这一图形-代码配置工具的eclipse。





关于 Clion 下的 STM32 开发，可以看看稚晖君的文章：https://zhuanlan.zhihu.com/p/145801160







这几年乐鑫公司推出的物联网芯片系列

可以简单理解为带蓝牙和WiFi的单片机



目前出货量已超过 10亿片



大学生比赛，

嘉立创的星火计划的项目里，

在招聘网站上的岗位还远比 ESP32， 



合宙的 ESP32-C3 和 ESP32-S3 系列





导航定位的程序计算量一般都特别大，很少在单片机裸机上跑，大多是用嵌入式 Linux，







## 八、MATLAB 程序组织和编译调试







## 九、Python 程序组织和编译调试







## 十、Fortran 程序组织和编译调试







## 相关网址

1. CMake入门实战：https://www.hahack.com/codes/cmake/#qmake
2. 谷歌C++风格指南：
3. C++ Standards Support in GCC：https://gcc.gnu.org/projects/cxx-status.html
4. Bjarne Stroustrup（C++之父）的个人主页：https://www.stroustrup.com/





## 写在最后

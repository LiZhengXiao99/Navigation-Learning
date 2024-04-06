> 转自西北工业大学布树辉老师的仓库：https://gitee.com/pi-lab/learn_slam

#  一步一步学SLAM

SLAM的全称是Simultaneous Localization and Mapping，同时定位与地图构建，就是通过激光或者视觉等传感器扫描环境，构建环境地图并同时给自己定位。这项技术是人工智能的感知部分，就是通过传感器将环境重建成数字模型并定位，从而实现AR、无人驾驶、自主探险等等应用。虽然经过了多年的发展，视觉SLAM取得了很大的进步，已逐步从实验室走向了应用，但是视觉SLAM仍然面临很多的挑战，例如动态环境下如何鲁邦的重建环境、如何实现与语义分割等有机融合、如何实现人类使用认知的方式完成精密三维重建等等。

##  1. 为何学习SLAM？

随着机器人、AR、自动驾驶的爆发式增长，SLAM方向的需求量仍在上升，而相关从业者相对较少，具备扎实基础知识，强大编程实现能力的人更少。目前对SLAM技术需求强烈的公司包括：互联网公司如百度、腾讯、阿里、京东等，计算机视觉算法公司如旷世、虹软、商汤等，自动驾驶创业公司如图森、momenta、景驰、驭势、滴滴及各大汽车厂商等，无人机/机器人公司如大疆、思岚、高仙等，AR移动终端应用相关公司如三星、华为、悉见等。

SLAM的实现是一类复杂的程序，包括数学、操作系统、多线程、C++、STL、CMake、大量第三方库等等，因此通过学习SLAM能够掌握复杂软件系统的设计、程序高级技巧，有了这些能力与技巧，后续做其他项目就会变得简单。**因此学习SLAM对[《个人的综合能力》](https://gitee.com/pi-lab/learn_slam/blob/master/Targets.md)的提升有着非常显著的作用，希望大家能够脚踏实地、一步一步去学习。**

## 2. 为什么学习SLAM比较难？

1. 深度学习在检测、识别领域具有强大的能力，但是在涉及**多视角几何相关的SLAM领域，深度学习的作用仍然有限**。究其原因是因为在多视角几何为基础的SLAM领域，需要明确清晰的理论基础保证，而深度学习的「黑盒子」模型目前还不是特别有效。
2. **SLAM技术门槛较高**。深度学习爆发后，很多非计算机视觉领域的从业者纷纷转而学习深度学习，由于深度学习本身黑盒子的特点，很多从业者不需要了解图像处理、计算机视觉的基础知识便可以得到一个相对较好的结果，因此入门门槛并不是很高。而SLAM需要非常多的数学、优化、数值计算、编程等技能，程序需要手工设计与实现，细节比较多，因此会卡住很多人。学习SLAM需要具备三维空间刚体变换、相机成像模型、特征点提取与匹配、多视角几何、光束平差、数值优化、程序优化等内容。这对于非该领域的从业者来说还是具有较高的门槛。
3. **需要高超的编程水平**：SLAM是一类比较复杂的软件系统，包括了大量的基础知识和技巧，主要有操作系统、多线程、C++编程、OpenGL。此外由于SLAM是一类实时处理的程序，因此对程序的性能优化有非常高的要求，因此比类似的技术structure from motion （SfM）难度高了不少。

## 3. 学习内容和需要达到的目标

SLAM的入门一直都是很多初学者的噩梦，但请不用怕，时至今日，它的理论和实践参考都已经非常完善，了解他们对于[《个人的综合能力》](https://gitee.com/pi-lab/learn_slam/blob/master/Targets.md)提升非常大，可以将SLAM相关的知识分为以下几类：

1. 几何理论： 多视图几何是认识SLAM中元素组成及其关联的核心，了解几何之后，即使不明白其具体的优化计算处理，也至少可以明白模块之间的输入输出，从而对大问题进行分解，是SLAM学习最基础的知识；
2. 编程： Linux系统的使用及程序工程能力是SLAM实践的关键，正确使用工具能让你少走弯路；
3. 计算机视觉（CV）：SLAM的前端主要作用是获取高质量的观测信息，决定了系统鲁棒性，区别与SLAM专属的数学及图优化处理，前端的理解往往比较直观，但涉及大量的工程处理；
4. 数学及优化： SLAM的后端决定了系统的整体精度，它涉及较多的数学，需要一定的耐心和钻研；

## 4. 如何学习SLAM？

### Stage0: 基础知识学习（[C++](https://gitee.com/pi-lab/learn_programming)，[Linux](https://gitee.com/pi-lab/learn_programming/tree/master/6_tools/linux)）

- C++
  - 大部分的SLAM程序通过C++实现，由于需要很高的执行效率，因此大部分的程序使用了较多的C++技巧
  - 需要考虑如何评估自己的C++能力是否适应SLAM编程的需求，如果自己的能力不够，该如何学习？从哪里找资料去学习？
  - 如果C++能力比较弱，可以参考[《一步一步学编程》](https://gitee.com/pi-lab/learn_programming)学习C++编程、数据结构与算法、编程项目等练习
- Linux
  - 由于大部分的SLAM系统开发都在Linux下面进行，因此非常有必要先把[Linux的基本操作](https://gitee.com/pi-lab/learn_programming/tree/master/6_tools/linux)学会
  - 基本的命令，如何安装软件包，如何查找软件包等等
  - 如何在Linux下编译软件，如何使用[CMake](https://gitee.com/pi-lab/learn_programming/tree/master/6_tools/cmake)
  - 如何使用Linux下面的IDE，例如QtCreator，KDevelop等等

### Stage1: [OpenCV学习](https://gitee.com/pi-lab/learn_slam/blob/master/1_OpenCV)

- [OpenCV的基本，`Mat`以及对应的基本操作](https://gitee.com/pi-lab/SummerCamp/tree/master/slam/cv#3--图像的基本操作)
- [读取图像，显示图像](https://gitee.com/pi-lab/SummerCamp/tree/master/slam/cv#4-数据获取与存储)
- [特征点提取，特征点匹配](https://gitee.com/link?target=https%3A%2F%2Fblog.csdn.net%2Fqq_38023849%2Farticle%2Fdetails%2F107309790)
- [相机标定原理与程序](https://gitee.com/pi-lab/SummerCamp/tree/master/slam/camera)
- Fundamental Matrix， Essential Matrix
- 简单的图像拼接（利用Homograph等）

### Stage2: SfM基本原理

- Structure from Motion原理，可以参考[《Mastering OpenCV with Practical Computer Vision Projects》的第四章](https://gitee.com/pi-lab/learn_slam/blob/master/references/MasteringOpenCV/MasteringOpenCV.pdf)
- 相机成像基本原理
- 坐标变换基本原理
- 最简单流程的程序理解，改进
- Bundle Adjust原理
- 如何编译示例程序: [SequenceSfM](https://gitee.com/pi-lab/SequenceSfM)
- 示例程序[SequenceSfM](https://gitee.com/pi-lab/SequenceSfM)存在什么问题？怎么改进？
- 在学习SfM过程，也可以看一下[《SLAM十四讲》](https://gitee.com/pi-lab/learn_slam/blob/master/references/视觉SLAM十四讲)

### Stage3: SLAM十四讲与相关库

- SLAM十四讲学习方法
  - 先快速把[《SLAM十四讲》](https://gitee.com/pi-lab/learn_slam/blob/master/references/视觉SLAM十四讲)过一遍
  - 主要是相机成像，三维的点如何变成二维上的点，把这个里面的公式搞通了，后面的一些东西就容易理解，好多公式是基于相机成像模型来进行推导的。
  - 在进行学习的同时可以配合[作业](https://gitee.com/pi-lab/learn_slam/blob/master/6_homework)达到练习的效果。

- 在看书学习的基础上，需要尝试自己独立编写处理模块，例如特征点提取，特征点匹配，初始化，三角化。。。

- SLAM相关库操作
  - Eigen3
  - G2O
  - [Ceres](https://gitee.com/pi-lab/learn_slam/blob/master/5_Ceres/README.md)
  - OpenGL
  - Qt
  - [Pangolin](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2Fstevenlovegrove%2FPangolin)

### Stage4：阅读并改进已有的SLAM系统

- 建议从 [VINS-Fusion](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2FHKUST-Aerial-Robotics%2FVINS-Fusion) 开始学习
- 如何编译程序？
- 这个程序分成几个模块，各个模块都完成什么工作，模块之间是如何衔接的？
- 能否将输入数据替换成自己录制的数据？能否把输出、显示改成自己的程序？
- 通过一步一步的仿真、增加算法模块，在具体任务的牵引下完成对SLAM的改进，具体参考：[SLAM练习开发项目](https://gitee.com/pi-lab/learn_slam/blob/master/4_Projects)

### Stage5：在消化吸收已有SLAM实现的基础上，编写自己的SLAM系统

- 在消化吸收已有SLAM程序的基础上，逐个模块替换成自己写的模块

## 5. 学习的建议

1. 不要想从空白一步到位自己独立编写出一个SLAM系统，SLAM是一个非常复杂的软件系统，包含了大量的数学、编程知识、编程技巧的系统，**需要先建立感性认识，然后熟悉各个技术模块，然后在有一点理解的基础上尝试集成、修改、改进等等**。
2. 由于SLAM包含了大量的数学知识、编程知识，因此强烈建议采用广度优先的方法来学习，即首先建立整体的感性认识；然后针对每一个技术模块了解其目的、意义、调用的方法等等；然后通过整合的方式去理解整体的衔接、组织。在不断深入理解的过程中，深入学习各个部分的数学原理、公式推导、公式如何转化成程序等等。否则一上来扎入复杂的公式，很有可能迷失方向。
3. **需要大量的编程练习！！！需要大量的编程练习！！！**SLAM其实真正难的并不完全是数学，通过库函数调用其实已经屏蔽了数学基本原理。最难的是需要比较高的编程技能，才能驾驭SLAM这类复杂的软件系统。因此如果自己觉得自己编程内功还欠火候，强烈建议同步练习编程的基本，自己设想一些不是太复杂，但是比较综合的小项目来练手。例如[《Learn Programming》中的Stage4，Stage5的题目](https://gitee.com/pi-lab/learn_programming)，通过项目、代码重构学习如何分析、处理比较复杂的问题。将这些知识与经验迁移到SLAM上，就是的学习SLAM程序没有那么难。
4. 强烈建议各位可以从structure from motion (SfM) 开始学习，因SfM是批处理的方式，方便理解每一个步骤的操作。通过构建比较好的输入数据，可以单独实现、测试每一个步骤。

前辈们的经验分享：

- [2013年进实验室 赵勇](https://gitee.com/pi-lab/learn_slam/blob/master/experiences/zhaoyong2021.md)

## 6. 参考资料

### 6.1 SLAM实现

- [VINS-Fusion](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2FHKUST-Aerial-Robotics%2FVINS-Fusion)
- [PI-SLAM](https://gitee.com/pi-lab/pi-slam)
- [SequenceSfM](https://gitee.com/pi-lab/SequenceSfM)
- [ORB-SLAM](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2Fraulmur%2FORB_SLAM), [ORB-SLAM2](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2Fraulmur%2FORB_SLAM2)

### 6.2 教程等

- [《一步一步学编程》](https://gitee.com/pi-lab/learn_programming)
- [《飞行器智能感知与控制实验室-暑期培训教程》](https://gitee.com/pi-lab/SummerCamp) , [（视频）](https://gitee.com/link?target=https%3A%2F%2Fwww.bilibili.com%2Fvideo%2FBV1dq4y1M7i8%2F)
- [90分钟学Python](https://gitee.com/pi-lab/machinelearning_notebook/tree/master/0_python)
- [《一步一步学ROS》](https://gitee.com/pi-lab/learn_ros)

### 6.3 图书与学习资料

- 参考图书
  - 《视觉SLAM十四讲》
  - 《计算机视觉中的多视图几何》（《Multiple View Geometry in Computer Vision》）
  - 《机器人学中的状态估计》
  - 《STL源码剖析》
  - 《Effective C++》
  - 《ROS机器人开发实践》
  - 《概率机器人》
- 学习资料
  - [SLAM从入门到放弃——学习SLAM 学习机器人 书籍推荐](https://gitee.com/link?target=https%3A%2F%2Fblog.csdn.net%2Fabcwoabcwo%2Farticle%2Fdetails%2F90377887)
  - [聊聊这两年学习slam啃过的书！](https://gitee.com/link?target=https%3A%2F%2Fzhuanlan.zhihu.com%2Fp%2F293039582)

### 6.3 其他参考资料

- [高翔博士的SLAM思维导图](https://gitee.com/pi-lab/learn_slam/blob/master/images/SLAM_AllInOne.png)
- [Linux](https://gitee.com/pi-lab/learn_programming/tree/master/6_tools/linux)
- [CMake](https://gitee.com/pi-lab/learn_programming/tree/master/6_tools/cmake)
- [编程代码参考、技巧集合](https://gitee.com/pi-lab/code_cook)：可以在这个代码、技巧集合中找到某项功能的示例，从而加快自己代码的编写
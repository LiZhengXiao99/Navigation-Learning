[TOC]

## 一、稠密化 ORB-SLAM3

ORB-SLAM3 生成的是稀释点云地图，只能定位，无法直接用于导航，需要对稀疏点云稠密化构建稠密地图。处理分为两大类：基于数据结构、基于机器学习。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690961951185.png" alt="1690961951185" style="zoom: 33%;" />

### 1、八叉树地图

把三维空间建模成许多小方块（体素），构建占据栅格地图（Occupancy Grid Map），节点存储它是否被占据的信息。学过地信的都知道这种地图编码方式比较高效，节省空间，当某个方块的子节点都相同，就无需展开。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690962321093.png" alt="1690962321093" style="zoom: 50%;" />

对于占据和空白来说，选择用概论的形式表达，这样就可以动态建模地图中的障碍物信息。

设 $y \in \mathbb{R}$ 为概率对数值， $x$ 为 0 到 1 之间的概率，则它们之间的变换为：$y=\operatorname{logit}(x)=\log \left(\frac{x}{1-x}\right)$，反变换为：$x=\operatorname{logit}^{-1}(y)=\frac{\exp (y)}{\exp (y)+1}$。当 $y$ 从 $-\infty \rightarrow+\infty$ 时， $x$ 相应地从 $0 \rightarrow 1$ 。 当 $y$ 取 0 的时， $x$ 取到了 0.5 。

因此，假设 $y$ 来表达节点是否被占据，当不断观测到占据时，让 $y$ 增加一个值，否则就减小一个值。 当查询概率时，再用逆 logit 变换，将 $y$ 转换为概率即可。用数学形式表示，设某节点为 $n$ ，观测数据为 $z$ 。那么从开始到 $t$ 时刻某点的概率对数值为 $L\left(n \mid z_{1: t}\right) ， t+1$ 时刻为：
$$
L\left(n \mid z{1: t+1}\right)=L\left(n \mid z{1: t-1}\right)+L\left(n \mid z_{t}\right)
$$
若上式写成概率形式，而不是概率对数形式，则：
$$
P\left(n \mid z{1: T}\right)=\left[1+\frac{1-P\left(n \mid z{T}\right)}{P\left(n \mid z{T}\right)} \frac{1-P\left(n \mid z{1: T-1}\right)}{P\left(n \mid z_{1: T-1}\right)} \frac{P(n)}{1-P(n)}\right]^{-1}
$$
有了对数数据，就可以根据 RGB-D 数据，更新整个八叉树地图。假设在 RGB-D 图像中观测到某个像素带有深度 d，就说明在深度值对应的空间点上观测到了一个占据数据，并且从相机光心出发，到这个点的线段上都没有物体（否则会被遮挡）。

安装 Octomap，八叉树地图及其可视化工具 octovis 已经集成到仓库中，安装命令如下：

```bash
sudo apt-get install liboctomap-dev octovis
```

### 2、CodeMapping

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690963337908.png" alt="1690963337908" style="zoom:50%;" />

CodeMapping 的网络结构如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690964289530.png" alt="1690964289530" style="zoom: 50%;" />

- 上层网络采用的是 **U-Net**，输入为：**灰度图**、**稀疏深度 depth** 和**重投影误差 maps** 。深度depth 和重投影误差值归一化在 $[0,1]$ 之间，输入的三个数据可以 concat 为一个三通道的输入，输出为：**深度不确定图**。

- 下层网络是**变分自动编码器 ( VAE )**，网络会生成**隐编码 $c$** ，椆密的深度预测图 $D$ 和深度不确定性图 $b$ 。

- 损失函数由一个深度重构 loss 和一个 KL 散度 loss 构成。 其中深度重构loss定义为：
  $$
  \sum{\mathbf{x} \in \Omega} \frac{\left|D[\mathbf{x}]-D{g t}[\mathbf{x}]\right|}{b[\mathbf{x}]}+\log (b[\mathbf{x}])
  $$


效果如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690964411332.png" alt="1690964411332" style="zoom:33%;" />

## 二、语义 ORB-SLAM3

**语义 SLAM（Semantic SLAM）**指将**语义分割**、**目标检测**、**实例分割**等技术用于 SLAM 中，系统在建图过程中不仅仅获得**环境中的几何信息**，同时还可以**识别环境中独立的个体**，获取其位置、姿态和功能属性等语义信息，以应对复杂场景完成更高级的任务。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690963453614.png" alt="1690963453614" style="zoom: 33%;" />

语义 SLAM 的优势：

- 传统SLAM方法以**静态环境假设为前提**，而语义SLAM可以**预知移动物体 (人、汽车等) 的动态特性**；
- 语义SLAM中的**相似物体知识表示可以共享**，通过维护共享知识库提高SLAM系统的**可扩展性和存储效率**；
- 语义SLAM可实现**智能路径规划**，如机器人可以搬动路径中的可移动物体等实现路径更优；
- 语义SLAM中**包含物体的功能属性**，可以提高机器人**对环境的自主感知**和**人机交互**能力。
  语义ORB-SLAM3 : 将目标检则或图像分割算法与ORB-SLAM3相结合。

通常，主要的步骤包括：

1. 利用**目标检测或图像分割**算法，获取空间中**物体的2D标签**；
2. 使用**点云分割**算法，对**稀疏点云进行分割**(分类)；
3. 结合**含有2D标签的彩色图像**和**对应的深度图像**，融合**分割后的稀疏点云**，获得**稠密点云语义地图**；
4. 稠密点云语义地图**转换为八叉树地图**，减小了地图的**存储空间**，并便于移动机器人进行避**障和导航**。

## 三、ORB-SLAM3 与深度学习

人脑

### 1、深度学习与特征提取

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690964694741.png" alt="1690964694741" style="zoom: 50%;" />

### 2、深度学习与帧间估计

**帧间估计**也称为**视觉里程计**，是通过分析关联相机图像之间的多视几何关系确定机器人位姿与朝向的过程，可作为视觉 SLAM 的前端。相较于传统的基于稀疏特征或稠密特征，基于深度学习的帧间估计方法的优势在于 :

- 端到端的学习方式，**无需特征提取、特征匹配**，输入数据直接得结果。
- 无需复杂的多视图几何计算过程，**方法更直观简洁**；
- 训练好的模型，**运算速度快**，效率更高。













## 四、ORB-SLAM 与强化学习






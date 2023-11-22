![image-20231112215541779](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231112215541779.png)

![image-20231112215515922](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231112215515922.png)

* 输入：地图，雷达数据，里程计
* 输出：位姿





文件夹：

* map 处理地图，维护了一个地图
* pf（particle filter）就是算法的核心粒子滤波
  * **eig3**：对称矩阵的特征值分解
  * **pf_vector**：实现位姿向量加减乘
  * **pf_pdf**：高斯采样位姿
  * **pf_kdtree**：实现 KD 树的数据结构，将位姿和其权重保存在一个 kdtree 中，加速位姿查找计算。
  * **pf**：实现例子滤波的核心，采样点类 pf_sample_t、集群类 pf_cluster_t、粒子集合类 pf_sample_set_t、粒子滤波类 pf_t
* sensors 得到数据后计算粒子滤波的，其中odom用来将粒子一起平移，laser用来计算权重，sensor是一个虚类，也就是一个接口，规范下函数名而已。



AmclNode 类是所以功能的集合，Wiki上的每一个功能都是AmclNode的一个函数实现的，比如拿到地图后建立数据，提供全局定位服务等等。在这些函数当中，最为核心的一个函数是laserReceiced，实现了收到观测数据就调用pf等函数进行粒子滤波的权重更新，输出pose的估计，粒子重采样等。






















































<div align="center">
    <a name="Top"></a>
	<h1>GNSS算法解惑</h1>
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
    <a href="https://www.zhihu.com/people/dao-ge-92-60/"><img src="https://img.shields.io/badge/Zhihu-知乎-blue" /></a>
    <img src="https://komarev.com/ghpvc/?username=LiZhengXiao99&label=Views&color=0e75b6&style=flat" alt="访问量统计" />
</div>


<br/>

[TOC]

### 单系统单频算法，改进到多系统多频算法，需要额外进行哪些处理？

* 单系统拓展到多系统，需要估计系统间偏差 ISB，本质主要是各卫星系统间的时间偏差，因为各卫星系统时间由不同的组织来维持。





### 消电离层组合放大噪声，UOFC 组合缩小一半噪声，指的是伪距噪声还是载波相位噪声？







### 多频观测值的组合，应该关注什么？





### 伪距、载波、多普勒量测数据，静止和匀速直线运动分别有何特征？





### 伪距和载波都做为观测值，伪距量测噪声远大于载波，对定位解算能起作用吗？





### 多路径效应误差是不是主要针对伪距，而载波不需要考虑？





### 伪距率和多普勒等价吗？有能输出伪距率的接收机吗？

* 多普勒靠载波环输出的，伪距率可以是码环输出的，载波环比码环精度高

* 据说 RINEX 标准，有伪距率



### 伪距、载波、多普勒三者之间的耦合关系？

* **多普勒和载波相互影响**：多普勒频移和载波相位在接收机里都主要靠锁相环确定，多普勒频移误差大，锁相环跟踪不上载波载波就周跳了，反过来载波受干扰误差大，影响锁相环里鉴相结果，误差也会加到多普勒频移里



### 锁相环的噪声带宽是什么意思？与信噪比、载噪比的关系？





### 都说载噪比C/N0更能反应信号质量，为何接收机还是输出信噪比SNR？





### 如何分辨天线是否有源？





### 只能单频GPS的天线与全系统全频点区别？





### 载波相位和多普勒都是载波环输出的，为何拿来平滑伪距效果会有差异？





### 接收机内部如何保持时间？PPS秒脉冲信号如何生成？





### 伪距单点定位，不改正对流层电离层，只估计钟差和三维坐标，精度有多少？





### 预报星历和实时 SSR 改正，精度比较？实时 PPP 能否用预报星历实现？





### 双天线一般要求两个天线之间距离至少1~1.5米，短了效果会差多少？

双天线基线长度与航向角精度关系：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/%E5%8F%8C%E5%A4%A9%E7%BA%BF%E5%9F%BA%E7%BA%BF%E9%95%BF%E5%BA%A6%E4%B8%8E%E8%88%AA%E5%90%91%E8%A7%92%E7%B2%BE%E5%BA%A6%E5%85%B3%E7%B3%BB.png" alt="双天线基线长度与航向角精度关系" style="zoom:50%;" />

### RTKLIB 没做北斗二的伪距多路径延迟改正，改正后能提高多少精度？





### DCB、UDP、FCB、OSB 等等偏差改正参数分别表示什么？










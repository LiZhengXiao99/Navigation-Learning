<div align="center">
    <a name="Top"></a>
	<h1>GNSS定位算法：SPP、RTK、PPP、RTK-PPP</h1>
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
    <a href="https://www.zhihu.com/people/dao-ge-92-60/"><img src="https://img.shields.io/badge/Zhihu-知乎-blue" /></a>
    <img src="https://komarev.com/ghpvc/?username=LiZhengXiao99&label=Views&color=0e75b6&style=flat" alt="访问量统计" />
</div>

<br/>

伪距单点定位的基本原理就是用一台接收机同时接收四颗以上卫星信号，获取卫星到接收机之间的距离，根据空间后方交会的原理，构建伪距观测值和接收机位置间的方程组，解方程组得到接收机的位置。具体计算上还需要注意以下问题：

* 伪距观测值存在着很多的误差，有些可以**建立模型修正**（如：电离层延迟、对流层延迟、卫星钟差）、有些需要**作为参数**（如：接收机钟差）一并估计。伪距模型修正是在构建观测方程组之前，然后用修正后的伪距观测值计算观测残差。由于钟差作为参数，所以总的参数个数为 4。

* 由于卫星数多于4颗，观测方程数大于参数个数，方程组超定，可以用**最小二乘原理**求出满足残差平方和最小的解。

* 由于观测方程不是线性的，得先**线性化**，在近似坐标处展开，求设计矩阵、新息向量，然后才能用最小二乘、卡尔曼滤波解算。

* 由于各观测值的精度不同，引入权阵 P 来控制观测值对结果影响的权重，比如采用**高度角定权模型**。

* 解算完之后还要再求 $\sigma_0$、$\sigma_{dx}$、$\sigma_{dy}$、$\sigma_{dz}$,PDOP，来**衡量结果的精度**。
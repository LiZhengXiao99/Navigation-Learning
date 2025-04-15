<div align="center">
    <a name="Top"></a>
	<h1>IMU 上手</h1>
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
</div>
---

<br/>

> 推荐阅读或观看：
>
> * 计算机视觉life：
>
> * Jason漫谈：[IMU 简介和误差综述](https://zhuanlan.zhihu.com/p/659329350#/)
>
> * [mymymind](https://space.bilibili.com/37049168)：[【硬核】用PCB手搓一个陀螺仪](https://www.bilibili.com/video/BV15r421u72u)







![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-404d6cc7825dbbb266d50e432d148acf_1440w.webp)







IMU 的价格从几元钱到几百万的都有，

高精度的激光IMU组合导航常用于



转台用于 IMU 标定，一台要几十甚至上百万，



最后总结一下，对于一款 IMU，咱们应该重点关注的东西：

* **陀螺仪零偏**：影响惯导性能最重要的指标，引起三阶的定位误差，战术级 IMU 是 1°/h；
* **输出数据、模式配置的接口**：
* **是否输出增量数据**：
* **IMU 中心位置**：
* **有无温度补偿**：

* **是否支持外部触发同步**：
* **是否支持 PPS 时间同步**：
* **是否支持里程计数据输入**：
* **输出数据格式**：
* **能否输出时间戳**：
* **Allan 方差曲线图**：
* **是否支持姿态解算**：
* **手册是否完善**：
* **是否有测试数据**：
* **其它功能**：磁力计、



---

### 01-MPU6050



很多单片机教程的 IIC 实验就是读取 MPU6050 的数据，

> 推荐阅读：[Arduino教程：MPU6050的数据获取、分析与处理](https://www.geek-workshop.com/thread-15392-1-1.html)


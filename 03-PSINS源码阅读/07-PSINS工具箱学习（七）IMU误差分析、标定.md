> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]





这里的误差分析主要指陀螺静态漂移（加表零漂用时域平滑图即可imumeanplot），其它误差在标定中介绍。误差分析有时间序列、功率谱、小波分析等方法，但最好的方法还是Allan方差分析法，它比较全面，能给出除逐次启动零偏之外的大部分误差，作图直观(文献[2]8章)。

avar：针对一只陀螺作Allan方差分析；

avars：同时分析多只陀螺的Allan方差。









sysclbt：系统级标定，43维状态/3维速度误差量测(文献[2]10章)

imuclbt：对IMU数据标定修正处理；

clbtfile：标定参数文件读写；

clbtdiff：比较同一惯导两次标定之间的偏差。
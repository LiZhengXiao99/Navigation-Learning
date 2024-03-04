> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、



RTKRCV



### 如何学着写实时定位解算程序？

* RTKLIB 提供了界面程序 RTKNAVI、和命令行程序 RTKRCV 两个可以进行实时定位解算的程序；建议在看源码前，先学学这两个程序的使用先对实时定位解算有一些认识。
* 

* 实时定位解算主要需要关注以下几方面：
  * 数据怎么读进来？如何将数据流中的数据存到内存中？
  * 读进来的数据以什么形式组织？存到了哪些变量里？解算过的数据要不要丢弃？
  * 

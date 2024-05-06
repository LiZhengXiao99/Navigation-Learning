> 转自：https://zhuanlan.zhihu.com/p/111725538

* 大而全的两个综述：
  * [Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age](https://ieeexplore.ieee.org/abstract/document/7747236/)：这一篇2016 年的综述从（1）Long-Term SLAM 任务中的算法“鲁棒性”和“尺度适应”两方面 （2）地图的表示形式（分别从尺度图与语义图）（3）SLAM的新的理论工具和新传感器在SLAM应用（3）Active SLAM（一种以减少状态估计不确定性为需求的SLAM过程）和（4）有关深度学习在SLAM中的应用 几个方面，先阐述每一个方面的基本挑战和问题，接着指出未来可能的工作方向。在第一部分，该综述简单总结了SLAM的任务和其存在的必要性、研究进展与历史、建模方法（基于滤波和优化）和相关学科支持。在 [SLAM: 现在，未来和鲁棒年代（一）| 翻译](https://zhuanlan.zhihu.com/p/26075315) 处已经有简单翻译版本
  * [Simultaneous Localization and Mapping: A Survey of Current Trends in Autonomous Driving](https://ieeexplore.ieee.org/abstract/document/8025618/)：这篇2017年的综述主要通过分析KITTI上排名靠前的算法，介绍了SLAM方法在自动驾驶方面的局限性，然后讨论了如何减轻这些局限性，行文主要包括（1）从基于滤波和基于优化两个方面介绍常用状态估计技术（2）给出近年来的细分主题的综述及相关数据集说明（3）给出已有SLAM方法的介绍及拓展到“自动驾驶”领域引发的“局限”讨论（4）在单体SLAM中提升建图一致性的措施和讨论（5）以“在线集成”及“离线集成”为分类的有关多机SLAM技术的讨论（6）应用于自动驾驶大规模尺度的SLAM介绍以及最后的（7）总结，指出新技术（如深度学习）在SLAM中的应用及指出其他领域（航空）同样开始涉及SLAM技术。

* 两篇更早期的SLAM问题综述文章，第一部分比较像简单的入门教程，第二部分则是当时较新的方法的简介（2006）：
  * [Simultaneous localization and mapping: part I](https://ieeexplore.ieee.org/document/1638022)
  * [Simultaneous localization and mapping (SLAM): part II](https://ieeexplore.ieee.org/abstract/document/1678144/)

* 对基于滤波方法的SLAM的利与弊讨论（2008）：
  * [The slam problem:a survey.](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.163.6439&rep=rep1&type=pdf)

* 有关“基于滤波”和“基于优化”的方法的比较（2010与2012）：
  * [Real-time monocular SLAM: Why filter?](https://ieeexplore.ieee.org/abstract/document/5509636/)
  * [Visual SLAM: Why filter?](https://www.sciencedirect.com/science/article/pii/S0262885612000248)

* 都有Gamini Dissanayake参与的，涉及到现代SLAM中 “可观性”、“一致性”、“收敛性”以及计算效率和复杂度的两篇理论探讨文章（2011与2016）：
  * [A Review of Recent Developments in Simultaneous Localization and Mapping](https://ieeexplore.ieee.org/iel5/6027500/6037376/06038117.pdf)
  * [A critique of current developments in simultaneous localization and mapping](https://journals.sagepub.com/doi/abs/10.1177/1729881416669482)

* 涉及到“基于视觉的地点重定位”的调研（2016），主要介绍“重定位”问题中的 图像处理（特征和描述符）、地图表示和置信度估计：
  * [Visual Place Recognition: A Survey](https://ieeexplore.ieee.org/abstract/document/7339473)

* 涉及到“多机器人SLAM”的调研（2016）：
  * [Multiple-Robot Simultaneous Localization and Mapping: A Review](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21620)
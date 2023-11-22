

[TOC]

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-60686c1413c72a75b932de87e6165050_1440w.webp)

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-615510bf0fb64293dfb37e46d56c9847_1440w.webp)

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-e76be29ef2675ca2c66cb56fa43dd04b_1440w.webp)





**文件结构**：

* **plan_node**：全局规划的入口
* **planner_core**：global_planner 的核心，进行初始化，调用 A* 或者 Dijkstra 进行全局规划
* **quadratic_calculator**：二次逼近方式，常用
* **potential_calculator**：直接返回当前点代价最小值，累加前面的代价值
* **grid_path**：栅格路径，从终点开始找上下左右四个点中最小的栅格直到起点
* **gradient_path**：梯度路径，从周围八个栅格中找到下降梯度最大的点
* **orientation_filter**：进行方向滤波
























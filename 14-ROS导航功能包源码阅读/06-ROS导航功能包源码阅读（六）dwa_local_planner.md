





dwa_local_planner中定义了七个打分器：

* oscillation_costs_：运动打分判断，是否震荡，是：代价大_
* _obstacle_costs_：障碍物碰撞检测打分，碰到障碍物，代价值增大
* path_costs_：局部轨迹（根据当前速度外推出的轨迹）与局部路径（规划的路径）对比，局轨迹离局部路径的横向偏差小，其代价值就小_
* _goal_costs_：局部轨迹与局部路径的终点进行对比，希望距离小
* goal_front_costs_：局部轨迹与局部路径的最终点的朝向一致_
* _alignment_costs_：局部轨迹与局部路径的朝向一致
* twirling_costs_：机器人旋转不要太大。
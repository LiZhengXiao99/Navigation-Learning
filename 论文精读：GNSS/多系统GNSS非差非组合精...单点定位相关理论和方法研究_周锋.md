GAMP 作者的博士论文，里面很多算法在 GAMP 有体现，精读这篇论文，可以更好的理解 GAMP。

[TOC]

## 摘要

* 围绕“多系统 GNSS”、“非差非组合 PPP”、“GNSS 偏差”、“实时 PPP”等研究热点，开展多系统 GNSS 非差非组合 PPP 定位模型算法和实际应用研究

* 实时估计单站 GLONASS 伪距频间偏差 IFB，四种实时模型化伪距 IFB 的方案：

  * 忽略伪距 IFB
  * 模型化伪距 IFB 为频率数的线性或二次多项式函数、
  * 每颗 GLONASS 卫星估计一个伪距 IFB 参数

  推导了四种策略分别在单 GLONASS、GPS + GLONASS 非差非组合 PPP 模型中的具体表达形式。结果表明：

  * 考虑 GLONASS 伪距 IFB 可使单 GLONASS 和 GPS + GLONASS PPP 解的收敛速度提高 20%以上。
  * 考虑 GLONASS 伪距 IFB 显著提升单 GLONASS 动态 PPP 的定位精度。
  * 每颗 GLONASS 卫星估计一个伪距 IFB 参数的方案要优于另外三种方案，说明 GLONASS 伪距 IFB 并不严格遵守与频率数成线性或二次函数的关系。

* 系统间偏差 ISB 估计：利用随机游走或白噪声过程来模型化 ISB 的估计策略；ISB 不仅依赖于不同 GNSS 系统对应的接收机硬件延迟差异，还与不同 GNSS 钟差产品相应的钟差基准约束引入的时间差异有关。使用不同分析中心的精密星历、钟差

 


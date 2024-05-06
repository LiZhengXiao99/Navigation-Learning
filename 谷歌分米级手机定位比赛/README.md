<div align="center">
    <a name="Top"></a>
	<h1>谷歌分米级手机定位比赛</h1>
</div>
<div align="center">
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
    <a href="https://www.zhihu.com/people/dao-ge-92-60/"><img src="https://img.shields.io/badge/Zhihu-知乎-blue" /></a>
    <img src="https://komarev.com/ghpvc/?username=LiZhengXiao99&label=Views&color=0e75b6&style=flat" alt="访问量统计" />
</div>

<br/>

谷歌手机分米级定位比赛 (Google Smartphone Decimeter Challenge，简称：GSDC)， Android GPS 团队主办，受到美国导航协会 ION 赞助，从 2021 年举办了第一届开始，每年一届，到现在已经连续进行了三年

比赛的内容简单来说，







### 比赛链接

* 2021年：https://www.kaggle.com/competitions/google-smartphone-decimeter-challenge
* 2022年：https://www.kaggle.com/competitions/smartphone-decimeter-2022
* 2023年：https://www.kaggle.com/competitions/smartphone-decimeter-2023



### 相关论文

* Optimizing the Use of RTKLIB for Smartphone-Based GNSS Measurements，下载





### 相关博客





> ### 23年比赛说明
>
> #### 概述
>
> 本次竞赛的目标是确定智能手机 GNSS 定位精度的极限：可以精确到分米甚至厘米级。您将根据安卓手机的原始 GNSS 测量结果开发一个模型。
>
> 你们的工作将有助于产生更好的定位，在更精细的人类行为地理空间信息和移动互联网之间架起一座桥梁。此外，更精确的数据还能带来新的导航方法。
>
> #### 描述
>
> 精确的智能手机定位服务使我们今天使用的许多导航功能成为可能。然而，目前的手机只能提供 3-5 米的定位精度。特定车道的导航并不总是可行的，这可能导致错过出口或到达时间不准确。
>
> 机器学习模型可以提高全球导航卫星系统（GNSS）数据的准确性，让数十亿安卓用户获得更精细的定位体验。
>
> 谷歌精确定位团队是安卓系统的一部分，曾在2021年和2022年举办过智能手机十米挑战赛。今年，这项比赛再次致力于寻找智能手机 GNSS 定位精度方面的创新研究，以提高人们导航周围世界的能力。
>
> 您的作品将有助于把定位精度提高到亚米级，甚至厘米级。因此，安卓用户可以在拥堵时获得更好的车道级导航或拼车估算。除了汽车之外，更好的定位数据还能实现增强现实步行游览、通过手机进行精确农业等。
>
> #### 评价
>
> 根据第 50 和第 95 百分位数距离误差的平均值对提交的数据进行评分。每部手机每秒计算一次预测纬度/经度与地面真实纬度/经度之间的水平距离（以米为单位）。这些距离误差形成一个分布，从中计算出第 50 和第 95 百分位数误差（即第 95 百分位数误差是 95% 的距离误差较小的值，单位为米）。然后求出每部手机的第 50 和第 95 百分位数误差的平均值。最后，计算测试集中所有手机的平均值。
>
> #### 提交文件
>
> 对于样本提交中的每部手机和 UnixTimeMillis，您都必须预测经纬度。样本提交通常要求每秒预测一次，但如果有效的 GNSS 信号太少，可能会出现较大的间隙。提交文件应包含页眉，格式如下：
>
> ```
> phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees
> 2020-05-15-US-MTV-1_Pixel4,1273608785432,37.904611315634504,-86.48107806249548
> 2020-05-15-US-MTV-1_Pixel4,1273608786432,37.904611315634504,-86.48107806249548
> 2020-05-15-US-MTV-1_Pixel4,1273608787432,37.904611315634504,-86.48107806249548
> ```
>
> #### 时间线
>
> - 2023 年 9 月 12 日 - 开始日期。
> - 2024 年 5 月 16 日--报名截止日期。您必须在此日期之前接受比赛规则才能参赛。
> - 2024 年 5 月 16 日--团队合并截止日期。这是参赛者加入或合并团队的最后一天。
> - 2024 年 5 月 23 日--最终提交截止日期。
> - 2024 年 6 月 30 日--ION GNSS+2024 论文提交截止日期。
>
> 除非另有说明，所有截止时间均为相应日期的世界协调时晚上 11:59。竞赛主办方保留在必要时更新竞赛时间表的权利。
>
> #### 奖金
>
> - 一等奖：7,000 美元，获奖作者可免费注册 ION GNSS+ 2024 会议，获奖作者最多可在会议酒店住宿 4 晚，获奖作者还可获得 1700 美元参加 ION GNSS+ 2024 会议的奖励。
> - 二等奖：5,000 美元，获奖作者可免费注册 ION GNSS+ 2024 会议，获奖作者最多可在会议酒店住宿 4 晚，以及获奖作者出席 ION GNSS+ 2024 会议的 1700 美元奖励。
> - 三等奖：3,000美元，获奖作者可免费注册ION GNSS+ 2024会议，获奖作者最多可在会议酒店住宿4晚，并可获得1,700美元参加ION GNSS+ 2024会议的奖励。
>
> 请注意，获奖者无需授权其解决方案。但是，参赛者必须提供技术论文，并在 ION GNSS+ 2024 大会上注册和发表论文，才有资格获得奖金。
>
> **所有论文必须在ION GNSS+2024大会截止日期（2024年6月30日）前提交。**
>
> 奖金中的现金部分将在竞赛结束日期后六周内支付。会议出席奖励将在 ION GNSS+ 2024 会议结束后支付。
>
> #### 说明
>
> 挑战赛组织者感谢导航研究所（ION）在 ION GNSS+2023 会议上专门为竞赛设立了一个特别会议，赞助了会议注册费，并为竞赛优胜者支付了 2024 年会议的酒店住宿费用。挑战赛组织者感谢为获奖者提供机会，让他们在大会上展示自己的作品，并在著名的颁奖午宴上领取奖金。
>
> #### 引用 
>
> Ashley Chow, Dave Orendorff, Michael Fu, Mohammed Khider, Sohier Dane, Vivek Gulati. (2023). Google Smartphone Decimeter Challenge 2023. Kaggle. https://kaggle.com/competitions/smartphone-decimeter-2023
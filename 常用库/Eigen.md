[toc]



## 一、简介



* eigen官网链接：http://eigen.tuxfamily.org/index.php?title=Main_Page

* 文档：http://eigen.tuxfamily.org/dox/

### 头文件作用

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20200713002410801.png)

一般为了省事，可以直接导入`<Eigen/Dense>` 或者`#<Eigen/Eigen>`，表示姿态需要引入`<Eigen/Geometry>`



## 二、稠密矩阵和向量操作

Eigen 中的 matrix 和 vector 底层采用 Matrix<> 模板类表示，vector 是特殊的 matrix  

### 1、矩阵的定义

* 矩阵的定义

* 矩阵的初始化和赋值

* 获取矩阵大小：调用成员函数 rows()、cols()、size()
* 调整动态矩阵大小：resize()



### 2、矩阵的运算


















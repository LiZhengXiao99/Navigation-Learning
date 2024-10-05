> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、Earth 类：地球参数和坐标转换

![image-20230925182052936](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925182052936.png)

`Earth` 类里都是静态函数，使用的时候直接`类名::成员函数()`，文件的开头定义了一些椭球参数：

```cpp
/* WGS84椭球模型参数
   NOTE:如果使用其他椭球模型需要修改椭球参数 */
const double WGS84_WIE = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F   = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA  = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB  = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0 = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_E1  = 0.0066943799901413156; /* 第一偏心率平方 */
const double WGS84_E2  = 0.0067394967422764341; /* 第二偏心率平方 */
```

### 1、gravity()：正常重力计算

重力是万有引力与离心力共同作用的结果，随纬度升高离心力增大但引力减小、随高程升高引力减小，共同作用下重力的计算公式如下：

$$
g_{L}=9.7803267715 \times\left(1+0.0052790414 \times \sin ^{2} L-0.0000232718 \times \sin ^{2} 2 L\right) \\
+h\times(0.0000000043977311\times\sin ^{2} L-0.0000030876910891)+0.0000000000007211\times\sin ^{4} 2 L
$$

```cpp
static double gravity(const Vector3d &blh) {
    double sin2 = sin(blh[0]);
    sin2 *= sin2;
    return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
         blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
}
```

### 2、meridianPrimeVerticalRadius()：计算子午圈半径 RM、卯酉圈半径 RN

返回值是 `Vector2d`，第一个是子午圈主曲率半径 RM、第二个是卯酉圈主半径 RN：
$$
R_{M}=\frac{R_{e}\left(1-e^{2}\right)}{\left(1-e^{2} \sin ^{2} L\right)^{3 / 2}}、R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```cpp
static Eigen::Vector2d meridianPrimeVerticalRadius(double lat) {
    double tmp, sqrttmp;

    tmp = sin(lat);	
    tmp *= tmp;
    tmp     = 1 - WGS84_E1 * tmp;
    sqrttmp = sqrt(tmp);

    return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
}
```

### 3、RN()：计算卯酉圈主半径 RN

$$
R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```cpp
static double RN(double lat) {
   double sinlat = sin(lat);
   return WGS84_RA / sqrt(1.0 - WGS84_E1 * sinlat * sinlat);
}
```

### 4、cne()：n系(导航坐标系)到e系(地心地固坐标系)转换矩阵

$$
C_{e}^{n}=\left[\begin{array}{ccc}-\sin \varphi & 0 & \cos \varphi \\ 0 & 1 & 0 \\ -\cos \varphi & 0 & -\sin \varphi\end{array}\right]\left[\begin{array}{ccc}\cos \lambda & \sin \lambda & 0 \\ -\sin \lambda & \cos \lambda & 0 \\ 0 & 0 & 1\end{array}\right]=\\ \left[\begin{array}{ccc}-\sin \varphi \cos \lambda & -\sin \varphi \sin \lambda & \cos \varphi \\ -\sin \lambda & \cos \lambda & 0 \\ -\cos \varphi \cos \lambda & -\cos \varphi \sin \lambda & -\sin \varphi\end{array}\right]
$$

```cpp
static Matrix3d cne(const Vector3d &blh) {
    double coslon, sinlon, coslat, sinlat;

    sinlat = sin(blh[0]);
    sinlon = sin(blh[1]);
    coslat = cos(blh[0]);
    coslon = cos(blh[1]);

    Matrix3d dcm;
    dcm(0, 0) = -sinlat * coslon;
    dcm(0, 1) = -sinlon;
    dcm(0, 2) = -coslat * coslon;

    dcm(1, 0) = -sinlat * sinlon;
    dcm(1, 1) = coslon;
    dcm(1, 2) = -coslat * sinlon;

    dcm(2, 0) = coslat;
    dcm(2, 1) = 0;
    dcm(2, 2) = -sinlat;

    return dcm;
}
```

### 5、qne()：计算n系(北东地)到e系(ECEF)转换四元数

位置更新的时候，调用此函数根据上一时刻经纬度，得到上一时刻的 qne，然后 `qee * qne * qnn` 得到当前时刻的 qne，再调用下面的 blh() 得到经纬度。
$$
\boldsymbol{q}_{n}^{e}=\left[\begin{array}{c}\cos (-\pi / 4-\varphi / 2) \cos (\lambda / 2) \\ -\sin (-\pi / 4-\varphi / 2) \sin (\lambda / 2) \\ \sin (-\pi / 4-\varphi / 2) \cos (\lambda / 2) \\ \cos (-\pi / 4-\sin / 2) \sin (\lambda / 2)]\end{array}\right]
$$

```cpp
/* n系(导航坐标系)到e系(地心地固坐标系)转换四元数 */
static Quaterniond qne(const Vector3d &blh) {
    Quaterniond quat;

    double coslon, sinlon, coslat, sinlat;

    coslon = cos(blh[1] * 0.5);
    sinlon = sin(blh[1] * 0.5);
    coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
    sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

    quat.w() = coslat * coslon;
    quat.x() = -sinlat * sinlon;
    quat.y() = sinlat * coslon;
    quat.z() = coslat * sinlon;

    return quat;
}
```

### 6、blh()：从n系到e系转换四元数得到纬度和经度

位置更新的时候，通过算当前时刻 n 系到 e 系转换四元数 qne，然后调用此函数得到经纬度。

```cpp
/* 从n系到e系转换四元数得到纬度和经度 */
static Vector3d blh(const Quaterniond &qne, double height) {
    return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
}
```

### 7、blh2ecef()：大地坐标(经纬高)转地心地固坐标

$$
\begin{array}{l}x=\left(R_{N}+h\right) \cos L \cos \lambda \\ y=\left(R_{N}+h\right) \cos L \sin \lambda \\ z=\left[R_{N}\left(1-e^{2}\right)+h\right] \sin L\end{array}
$$

```cpp
/* 大地坐标(纬度、经度和高程)转地心地固坐标 */
static Vector3d blh2ecef(const Vector3d &blh) {
    double coslat, sinlat, coslon, sinlon;
    double rnh, rn;

    coslat = cos(blh[0]);
    sinlat = sin(blh[0]);
    coslon = cos(blh[1]);
    sinlon = sin(blh[1]);

    rn  = RN(blh[0]);
    rnh = rn + blh[2];

    return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * WGS84_E1) * sinlat};
}
```

### 7、ecef2blh()：地心地固坐标转大地坐标

$$
\begin{array}{c}B_{0}=\arctan \left(\frac{Z}{\left(1-e^{2}\right) p}\right) \\ N_{k}=\frac{a}{\sqrt{1-e^{2} \sin ^{2} B_{k-1}}} \\ H_{k}=\frac{p}{\cos B_{k-1}}-N_{k} \\ B_{k}=\arctan \left(\frac{z}{\left(1-\frac{e^{2} N_{k}}{N_{k}}\right)\ p }\right)\end{array}
$$

```cpp
static Vector3d ecef2blh(const Vector3d &ecef) {
    double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
    double rn;
    double lat, lon;
    double h = 0, h2;

    // 初始状态
    lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
    lon = 2.0 * atan2(ecef[1], ecef[0] + p);

    do {
        h2  = h;
        rn  = RN(lat);
        h   = p / cos(lat) - rn;
        lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
    } while (fabs(h - h2) > 1.0e-4);

    return {lat, lon, h};
}
```

### 8、DRi()： 计算 n 系相对位置转大地坐标相对位置的矩阵

通过算出来的矩阵实现 ENU 和 LLH 之间的转换：

* 捷联惯导求出的速度是 n 系的，要转成经纬度增量就得用这个矩阵。
* 杆臂误差补偿时也需要用这个函数，因为杆臂是 n 系的，算出的 IMU 坐标和给的 GNSS 解都是经纬高。
* 存的位置是经纬高，GNSS 量测更新时候计算的是 ENU 下位置的增量，反馈的时候也需要此矩阵。

$$
\left[\begin{array}{l}\delta \varphi \\ \delta L \\ \delta H\end{array}\right]=\left[\begin{array}{ccc}\left(R_{M}+H\right)^{-1} & 0 & 0 \\ 0 & \left(R_{N}+H\right)^{-1} & 0 \\ 0 & 0 & -1 \end{array}\right]\left[\begin{array}{l}\delta \boldsymbol{p}_{N} \\ \delta \boldsymbol{p}_{E} \\ \delta \boldsymbol{p}_{B}\end{array}\right]
$$

```cpp
/* n系相对位置转大地坐标相对位置 */
static Matrix3d DRi(const Vector3d &blh) {
    Matrix3d dri = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dri(0, 0) = 1.0 / (rmn[0] + blh[2]);
    dri(1, 1) = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
    dri(2, 2) = -1;
    return dri;
}
```

### 9、DR()：计算大地坐标相对位置转 n 系相对位置的矩阵

就是上面 `DRI()` 计算矩阵的倒数。
$$
\left[\begin{array}{l}\delta \varphi \\ \delta L \\ \delta H\end{array}\right]=\left[\begin{array}{ccc}\left(R_{M}+H\right) & 0 & 0 \\ 0 & \left(R_{N}+H\right) & 0 \\ 0 & 0 & -1 \end{array}\right]\left[\begin{array}{l}\delta \boldsymbol{p}_{N} \\ \delta \boldsymbol{p}_{E} \\ \delta \boldsymbol{p}_{B}\end{array}\right]
$$

```cpp
/* 大地坐标相对位置转n系相对位置 */
static Matrix3d DR(const Vector3d &blh) {
    Matrix3d dr = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dr(0, 0) = rmn[0] + blh[2];
    dr(1, 1) = (rmn[1] + blh[2]) * cos(blh[0]);
    dr(2, 2) = -1;
    return dr;
}
```

### 10、local2global()：局部坐标(在origin处展开)转大地坐标

在 `enwn()` 中被调用，为了方便能直接传入北东地（n 系）坐标计算 n 系相对于 e 系转动角速度在 n 系的投影。

```cpp
static Vector3d local2global(const Vector3d &origin, const Vector3d &local) {

    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = ecef0 + cn0e * local;
    Vector3d blh1  = ecef2blh(ecef1);

    return blh1;
}
```

### 11、global2local()：大地坐标转局部坐标(在origin处展开)

好像整个程序中都没用到这个函数。

```cpp
static Vector3d global2local(const Vector3d &origin, const Vector3d &global) {
    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = blh2ecef(global);

    return cn0e.transpose() * (ecef1 - ecef0);
}
```

```cpp
static Pose global2local(const Vector3d &origin, const Pose &global) {
    Pose local;

    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = blh2ecef(global.t);
    Matrix3d cn1e  = cne(global.t);

    local.t = cn0e.transpose() * (ecef1 - ecef0);
    local.R = cn0e.transpose() * cn1e * global.R;

    return local;
}
```

### 12、iewe()：地球自转角速度投影到e系

$$
\boldsymbol{\omega}_{i e}^{e}=\left[\begin{array}{lll}0 & 0 & \omega_{e}\end{array}\right]^{T}
$$

```cpp
static Vector3d iewe() {
    return {0, 0, WGS84_WIE};
}
```

### 13、iewn()：地球自转角速度投影到n系

$$
\boldsymbol{\omega}_{i e}^{n}=\left[\begin{array}{lll}\omega_{e} \cos \varphi & 0 & -\omega_{e} \sin \varphi\end{array}\right]^{T}
$$

```cpp
static Vector3d iewn(double lat) {
    return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
}
```

也可以直接传入北东地（n 系）坐标计算：

```cpp
static Vector3d iewn(const Vector3d &origin, const Vector3d &local) {
    Vector3d global = local2global(origin, local);
    return iewn(global[0]);
}
```

### 14、enwn()：n系相对于e系转动角速度投影到n系

由载体运动线速度和地球曲率引起，与东向、北向速度有关，与天向速度无关
$$
\boldsymbol{\omega}_{e n}^{n}=\left[\begin{array}{lll}\frac{v_{E}}{R_{N}+h} & \frac{-v_{N}}{R_{M}+h} & -\frac{v_{E} \tan \varphi}{R_{N}+h}\end{array}\right]^{T}
$$

```cpp
static Vector3d enwn(const Eigen::Vector2d &rmn, const Vector3d &blh, const Vector3d &vel) {
    return {vel[1] / (rmn[1] + blh[2]), -vel[0] / (rmn[0] + blh[2]), -vel[1] * tan(blh[0]) / (rmn[1] + blh[2])};
}
```

同样也可以直接传入北东地（n 系）坐标计算：

```cpp
static Vector3d enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel) {
    Vector3d global     = local2global(origin, local);
    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(global[0]);

    return enwn(rmn, global, vel);
}
```

## 二、Rotation 类：姿态转换

![image-20231020105459334](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231020105459334.png)

### 1、matrix2quaternion()：旋转矩阵转四元数

Eigen 中的四元数可以直接传入旋转矩阵（三维矩阵）构造：

```cpp
static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
    return Quaterniond(matrix);
}
```

### 2、quaternion2matrix()：四元数转旋转矩阵

四元数调用 `toRotationMatrix()` 函数，转为旋转矩阵：

```cpp
static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
    return quaternion.toRotationMatrix();
}
```

### 3、matrix2euler()：旋转矩阵转欧拉角

ZYX 旋转顺序，前右下的 IMU，输出 RPY：

```cpp
static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) {
    Vector3d euler;

    euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if (dcm(2, 0) <= -0.999) {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
    } else if (dcm(2, 0) >= 0.999) {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
    } else {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }

    // heading 0~2PI
    if (euler[2] < 0) {
        euler[2] = M_PI * 2 + euler[2];
    }

    return euler;
}
```

### 4、quaternion2euler()：四元数转欧拉角

先调用 `toRotationMatrix()` 转为旋转矩阵，再调用 `matrix2euler()` 转欧拉角：

```cpp
static Vector3d quaternion2euler(const Quaterniond &quaternion) {
    return matrix2euler(quaternion.toRotationMatrix());
}
```

### 5、rotvec2quaternion()：等效旋转矢量转四元数

根据传入的旋转矢量，计算向量的长度作为旋转的角度，计算向量的归一化版本作为旋转的轴，然后调用 `AngleAxisd()`，将角度和轴转换为四元数。

```cpp
static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
    double angle = rotvec.norm();       // 计算向量的长度作为旋转的角度
    Vector3d vec = rotvec.normalized(); // 计算向量的归一化版本作为旋转的轴
    return Quaterniond(Eigen::AngleAxisd(angle, vec));  // 调用 AngleAxisd()，将角度和轴转换为四元数
}
```

### 6、quaternion2vector()：四元数转旋转矢量

传入的四元数通过 Eigen::AngleAxisd 类的构造函数转换为角度轴（angle-axis）表示。角度轴是一个描述旋转的方法，其中旋转角度和旋转轴是两个独立的部分。然后，该函数返回这个角度轴表示的旋转的角度乘以旋转的轴，得到一个三维向量。这个向量的 x、y 和 z 分量分别对应于旋转轴在x、y 和 z 轴上的分量，而其长度（或者说范数）等于旋转角度。

```cpp
static Vector3d quaternion2vector(const Quaterniond &quaternion) {
    Eigen::AngleAxisd axisd(quaternion);
    return axisd.angle() * axisd.axis();
}
```

### 7、euler2matrix()：欧拉角转旋转矩阵

三个欧拉角分别转为 ZYX 角轴，相乘之后构造旋转矩阵

```cpp
static Matrix3d euler2matrix(const Vector3d &euler) {
    return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
}
```

### 8、euler2quaternion()：欧拉角转四元数

三个欧拉角分别转为 ZYX 角轴，相乘之后构造四元数

```cpp
static Quaterniond euler2quaternion(const Vector3d &euler) {
    return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
}
```

### 9、skewSymmetric()：计算三维向量反对称阵

```cpp
static Matrix3d skewSymmetric(const Vector3d &vector) {
    Matrix3d mat;
    mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
    return mat;
}
```

### 10、quaternionleft()、quaternionright()：四元数矩阵


$$
\boldsymbol{P} \circ \boldsymbol{Q}=\left[\begin{array}{cccc}p_{0} & -p_{1} & -p_{2} & -p_{3} \\ p_{1} & p_{0} & -p_{3} & p_{2} \\ p_{2} & p_{3} & p_{0} & -p_{1} \\ p_{3} & -p_{2} & p_{1} & p_{0}\end{array}\right]\left[\begin{array}{l}q_{0} \\ q_{1} \\ q_{2} \\ q_{3}\end{array}\right]=\boldsymbol{M}_{P} \boldsymbol{Q}=\left[\begin{array}{cccc}q_{0} & -q_{1} & -q_{2} & -q_{3} \\ q_{1} & q_{0} & q_{3} & -q_{2} \\ q_{2} & -q_{3} & q_{0} & q_{1} \\ q_{3} & q_{2} & -q_{1} & q_{0}\end{array}\right]\left[\begin{array}{l}p_{0} \\ p_{1} \\ p_{2} \\ p_{3}\end{array}\right]=\boldsymbol{M}_{Q}^{\prime} \boldsymbol{P}
$$

$$
\boldsymbol{M}_{P}=\left[\begin{array}{cccc}p_{0} & -p_{1} & -p_{2} & -p_{3} \\ p_{1} & p_{0} & -p_{3} & p_{2} \\ p_{2} & p_{3} & p_{0} & -p_{1} \\ p_{3} & -p_{2} & p_{1} & p_{0}\end{array}\right]=\left[\begin{array}{cc}p_{0} & -\boldsymbol{p}_{v}^{\mathrm{T}} \\ \boldsymbol{p}_{v} & p_{0} \boldsymbol{I}+\left(\boldsymbol{p}_{v} \times\right)\end{array}\right]
$$

```cpp
static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
    Eigen::Matrix4d ans;
    ans(0, 0)             = q.w();
    ans.block<1, 3>(0, 1) = -q.vec().transpose();
    ans.block<3, 1>(1, 0) = q.vec();
    ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
    return ans;
}
```

$$
\boldsymbol{M}_{Q}^{\prime}=\left[\begin{array}{cccc}q_{0} & -q_{1} & -q_{2} & -q_{3} \\ q_{1} & q_{0} & q_{3} & -q_{2} \\ q_{2} & -q_{3} & q_{0} & q_{1} \\ q_{3} & q_{2} & -q_{1} & q_{0}\end{array}\right]=\left[\begin{array}{cc}q_{0} & -\boldsymbol{q}_{v}^{\mathrm{T}} \\ \boldsymbol{q}_{v} & q_{0} \boldsymbol{I}-\left(\boldsymbol{q}_{v} \times\right)\end{array}\right]
$$

```cpp
static Eigen::Matrix4d quaternionright(const Quaterniond &p) {
    Eigen::Matrix4d ans;
    ans(0, 0)             = p.w();
    ans.block<1, 3>(0, 1) = -p.vec().transpose();
    ans.block<3, 1>(1, 0) = p.vec();
    ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
    return ans;
}
```


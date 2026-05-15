# TF变换

### 坐标系
一台机器人通常会需要用到多个坐标系，例如：机器人本体坐标系（BaseLink）、相机坐标系（Camera Link）、枪管坐标系（Shoot Link）、激光雷达坐标系（Laser Link）等。
每个坐标系都有自己的原点和轴向。

### 位姿
**位姿** 就是 **位置** 和 **姿态**。

位置就是三维坐标，而姿态是描述一个物体相对于参考坐标系的朝向的方式。

最简单的：
2维地图：位置：$(x, y)$，姿态：朝向北方
3维空间：位置：$(x, y, z)$，姿态：yaw，pitch，roll（欧拉角）

姿态的表示方法有很多，一般可以用四元数、旋转矩阵、旋转向量和欧拉角等来表示，这几种表示方法都各有优缺点。

### 点的TF变换
TF变换是指同一个物体（点、向量、位姿）在不同坐标系下的转换关系。
简单来说：**把同一个物体，从A坐标系表示 → 换成B坐标系表示。**

只要已知两个坐标系之间的变换关系，==**旋转 + 平移**==，就可以把同一个点/位姿在不同坐标系之间进行转换。

我们假设：
- 有两个坐标系：A 和 B
- 一个**点P**在 A 坐标系下的坐标是 $P_a$（x，y，z）
- 我们想求这个点在 B 坐标系下的坐标 $P_b$。

我们需要两个信息：

1. **A 坐标系相对于 B 坐标系的位置（平移）** → 一个向量 $\mathbf{t}_{BA}$  
（A的原点在B坐标系中的位置，所以必须先把点转到 B 的方向体系里，才能正确加上 B 坐标系里的平移）
1. **A 坐标系相对于 B 坐标系的朝向（旋转）** → 一个旋转矩阵 $\mathbf{R}_{BA}$
（R 的每一列，表示：A坐标系的坐标轴，在B坐标系中的表示）

$$
\mathbf{P}_b = \mathbf{R}_{BA} \cdot \mathbf{P}_a + \mathbf{t}_{BA}
$$

### 那么加上姿态的TF变换就是

我们假设：

- 两个坐标系：A 和 B  
- 一个物体 $P$，在 **B 坐标系下**的：
  - 位置是 $\mathbf{p}_B$
  - 姿态是一个旋转矩阵 $\mathbf{R}_B^P$（表示物体自身坐标轴相对B坐标轴的旋转）
- 我们知道 **B 相对于 A** 的变换关系：
  - 平移：$\mathbf{t}_A^B$
  - 旋转：$\mathbf{R}_A^B$
  
#### 那么在 A 坐标系下的位姿就是：

#### ✅ 位置变换：

$$
\mathbf{p}_A = \mathbf{R}_A^B \cdot \mathbf{p}_B + \mathbf{t}_A^B
$$

#### ✅ 姿态变换：

$$
\mathbf{R}_A^P = \mathbf{R}_A^B \cdot \mathbf{R}_B^P
$$

姿态直接乘上旋转矩阵(左乘)，位置还是先旋转后平移，别搞反啦



**如何得到旋转矩阵**
- 通过标定得到，比如solvepnp
- 通过IMU，里程计
- 手动测量



### 旋转矩阵
在三维空间中，旋转矩阵用于描述一个物体或坐标系如何相对于另一个坐标系旋转。它是一个 3×3 的正交矩阵，用于表示姿态。

旋转矩阵满足以下性质：
- 行向量（或列向量）两两正交
- 每个向量长度为1
- 行列式为 +1
- $R^{-1} = R^T$（即转置等于逆）

以下列出了绕 $x$ 轴、$y$ 轴、$z$ 轴旋转的标准旋转矩阵：

> 正方向：右手握住轴，大拇指指向正方向，其余四指方向为正转角方向。

### 绕 X 轴旋转 $\theta$

$$
R_x(\theta) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & -\sin\theta \\
0 & \sin\theta & \cos\theta
\end{bmatrix}
$$

### 绕 Y 轴旋转 $\theta$

$$
R_y(\theta) =
\begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{bmatrix}
$$

### 绕 Z 轴旋转 $\theta$

$$
R_z(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$



---

#### 欧拉角(最简单常用)
我们在生活中描述旋转时常说“先绕X轴转个角度，再绕Y轴，再绕Z轴”，这就是**欧拉角**的基本思想
称为roll、pitch、yaw。

base_link坐标系左右旋转：yaw在变化
base_link坐标系抬头低头：pitch在变化
base_link坐标系翻滚：roll在变化

用一个三步过程描述：
1. **第一步：绕Z轴旋转 $\gamma$（Yaw）**
2. **第二步：绕Y轴旋转 $\beta$（Pitch）**
3. **第三步：绕X轴旋转 $\alpha$（Roll）**

最终的旋转矩阵为：

$$
R = R_x(\alpha) \cdot R_y(\beta) \cdot R_z(\gamma)
$$

==每个轴的基本旋转矩阵为：==

- 绕X轴（Roll）：
  $$
  R_x(\alpha) = \begin{bmatrix}
  1 & 0 & 0 \\
  0 & \cos\alpha & -\sin\alpha \\
  0 & \sin\alpha & \cos\alpha
  \end{bmatrix}
  $$

- 绕Y轴（Pitch）：
  $$
  R_y(\beta) = \begin{bmatrix}
  \cos\beta & 0 & \sin\beta \\
  0 & 1 & 0 \\
  -\sin\beta & 0 & \cos\beta
  \end{bmatrix}
  $$

- 绕Z轴（Yaw）：
  $$
  R_z(\gamma) = \begin{bmatrix}
  \cos\gamma & -\sin\gamma & 0 \\
  \sin\gamma & \cos\gamma & 0 \\
  0 & 0 & 1
  \end{bmatrix}
  $$

>你会发现和上述旋转矩阵的描述其实完全相同^-^因为roll、pitch、yaw理解为是X、Y、Z轴。

#### 四元数（ROS2中的TF表示）
四元数是一种复数系统的扩展，由一个实部和三个虚部组成，通常表示为：
**q = a + bi + cj + dk**
其中 a、b、c、d 是实数，而 i、j、k 是三个特殊的虚数单位。满足如下关系:

$$
i^2 = j^2 = k^2 = ijk = -1
$$

四元数的加法和乘法定义如下：

$$
q_1 + q_2 = (w_1 + w_2) + (x_1 + x_2)i + (y_1 + y_2)j + (z_1 + z_2)k
$$

$$
q_1 \cdot q_2 = (w_1w_2 - x_1x_2 - y_1y_2 - z_1z_2) + (w_1x_2 + x_1w_2 + y_1z_2 - z_1y_2)i + (w_1y_2 - x_1z_2 + y_1w_2 + z_1x_2)j + (w_1z_2 + x_1y_2 - y_1x_2 + z_1w_2)k
$$

四元数的模定义如下：

$$
|q| = \sqrt{w^2 + x^2 + y^2 + z^2}
$$

四元数的共轭定义如下：

$$
q^* = w - xi - yj - zk
$$

四元数的逆定义如下：

$$
q^{-1} = \frac{q^*}{|q|^2}
$$

##### 四元数表示旋转（解决了欧拉角的万向锁问题）

一个单位四元数（模长为1）可以表示**绕任意轴旋转任意角度**。

模长不为1的话，旋转后的结果向量会被**缩放**。

如果要绕单位轴 $\hat{\mathbf{n}} = [n_x, n_y, n_z]^T$ 旋转角度 $\theta$，那么对应的四元数是：

$$
\mathbf{q} = \begin{bmatrix}
\cos(\theta/2) \\
n_x \cdot \sin(\theta/2) \\
n_y \cdot \sin(\theta/2) \\
n_z \cdot \sin(\theta/2)
\end{bmatrix}
$$

> 注：是 $\theta/2$，不是 $\theta$ ，别搞错了！

四元数的主要用途就是：**旋转点、表示姿态**等。

##### 旋转一个向量 $\mathbf{v}$：

1. 把向量 $\mathbf{v}$ 变成四元数形式：$\mathbf{v}_q = [0, v_x, v_y, v_z]$
2. 用四元数旋转它：

$$
\mathbf{v}' = \mathbf{q} \cdot \mathbf{v}_q \cdot \mathbf{q}^{-1}
$$

其中 $\mathbf{q}^{-1}$ 是四元数的共轭（因为它是单位四元数）：

$$
\mathbf{q}^{-1} = \begin{bmatrix} w \\ -x \\ -y \\ -z \end{bmatrix}
$$


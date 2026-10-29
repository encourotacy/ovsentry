> 这是一个对tasks/auto_buff/文件夹下程序的简要说明文件
[TOC]

---
### yolo11_buff
&emsp;&emsp;这是一个调用yolo11模型识别图中目标的程序。整个过程是比较线性的：
- 先获取路径、加载模型；
- 然后缩放并转换图片，将图片写入输入层；
- 让模型进行推理（这个只有一行, `infer_request.infer()`）
- 推理后读取和整理输出层的信息。

&emsp;&emsp;相比于同济原版，我们在读取输出层部分这里添加了`checkOutputLayout()`等函数以兼容更多输出格式的模型。

#### (1) 缩放图片中，`cv::warpAffine`的解释
> 内容部分来自 [opencv3.0文档](https://docs.opencv.org/3.0.0/da/d54/group__imgproc__transform.html#ga0203d9ee5fcd28d40dbc4a1ea4451983)，只介绍了函数的基本功能。
```cpp
void cv::warpAffine (   InputArray  src,
                        OutputArray dst,
                        InputArray  M,
                        Size 	dsize)
```
将图像`src`仿射变换成`dst`。`M`是一个 $2\times 3$ 的矩阵，变换规则如下：
$$\text{dst}(x,y) = \text{src}(M_{11}x+M_{12}y+M_{13},M_{21}x+M_{22}y+M_{23})$$

程序中，$M = \begin{matrix} \text{scale}&0&0 \\ 0&\text{scale}&0 \end{matrix}$，那么变换的规则就是 $\text{dst}(x,y) = \text{src}(\text{scale}\times x,\text{scale}\times y)$，即**将图像按照`scale`比例缩放（不改变宽高比），放置在左上角，不足的部分会由黑色填充**，

---
### buff_solver
&emsp;&emsp;调用opencv的PnP解算，计算出符的各种位置的世界坐标。
&emsp;&emsp;有关基变换的知识可以看看[3b1b的《线性代数的本质 - 09 - 基变换》](https://www.bilibili.com/video/BV1Ls411b7r2/?vd_source=badebcc26d529d40c84f1a34dcb48378)。
&emsp;&emsp;这里的 R 表示旋转矩阵，t 表示平移向量，其名字表示的是**它们参与运算时的效果**。比如说，R_buff2camera就是和向量相乘后，使得能量机关坐标系下的坐标变成相机坐标系下的坐标，t_camera2gimbal_就是在将相机坐标系下的坐标变成云台坐标系下的坐标时，需要加的偏移。
&emsp;&emsp;以下是各个坐标系：
- 能量机关坐标系：可以根据OBJECT_POINTS推测出，
  - 原点：能量机关中心
  - XYZ轴正方向：前/左/上
- 相机坐标系：来自OpenCV的硬性规定，
  - 原点：相机的**光心**，光线通过该点时传播方向不发生改变，比如凸透镜的中心。
  - XYZ轴正方向：右/**下**/前
- 云台坐标系：也是规定
  - 原点：云台的**旋转中心**。我们的云台有yaw轴（沿着竖直的线转动）和pitch轴（沿着水平的线转动）两个方向旋转，而这两个条线通常是相交的（至少步兵上是这样的），它们的**交点**就是云台的旋转中心。
  - XYZ轴方向：前后/左右/上下（正方向还不清楚(悲)，之后可能要试试）
- 世界坐标系：这是IMU指定的绝对坐标系，具体是怎么样的其实不重要，我们只需要知道这个坐标只在开机的时候确定、之后的运算中不会变动就行。

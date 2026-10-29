> 这是一个对tools/文件夹下程序的简要说明文件
[TOC]

---
### thread_safe_queue.hpp
&emsp;&emsp;这是一个线程安全的队列类，使用了大量的互斥锁保证数据不会被同时操作导致混乱，同时使用了std::condition_variable在队列为空时阻塞线程，直到有新的数进入队列。
&emsp;&emsp;这个队列在ovgimbal中用于大量存储imu数据，也在recorder里利用其特性存储图像数据。

---
### recorder
&emsp;&emsp;这是一个内录程序，接收图像和四元数并把它们分别保存为.avi和.txt文件，以供之后回放(回放程序是test/auto_aim_test.cpp和test/auto_buff_test.cpp)。

---
### ransac_sine_fitter
&emsp;&emsp;这是一个抗干扰的正弦波拟合器，使用随机抽样一致算法（Random sample consensus，RANSAC）过滤掉过于不合理的观测值以预测大符的速度。
但是不知道为什么，我们的自瞄里没有使用这个东西。
#### 随机抽样一致算法（Random sample consensus，RANSAC）
可以看看这个[博客](https://www.cnblogs.com/xingshansi/p/6763668.html)
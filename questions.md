
# 问题（重要）：
1. aimer的判断逻辑是怎么样的？是否可以在planner上使用？

# 问题（常规）：
1. ovgimbal的说明：球面插值是什么？
2. 为什么yolo11_buff的模型要用专门指定用CPU来推理？
3. yolov5中保留了一个postprocess的接口，似乎是要用于异步推理的，这个是什么东西？
4. armor的yolov5初始化中，为什么没有初始化priority？

# 问题（不重要）：
1. camera类是如何工作的？
2. logger的原理？
3. yolo11是如何工作的？输入层、输出层的大小是如何设置的？
4. math_tools的eular函数的原理
5. RANSAC的原理？这个算法直接用太不稳定了吧...我需要去看看论文已验证其合理性？
6. `capture_chessboard.cpp`中，`cornerSubPix` 有一个迭代限制条件 `cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01)`，这里的值是怎么确定的
  - 函数说明文档详见 ![这里](https://docs.opencv.org/5.0/main_modules/imgproc_feature.html#cornersubpix)

# todo:
1. 复制整个文档，并将其中的注释以Doxygen格式重写...（建议先AI生成，然后再校对）
2. 写前哨建模
3. 写哨兵通信和了解大小yaw怎么计算。


附件中是tasks/auto_buff 文件夹下的yolo11_buff程序，这里使用了yolo11模型识别另一种目标。但是这里有很多与yolov5相比不同的写法：
1. yolo11 的 input_tensor_ 来自于构造方法中的infer_request_；但是yolov5中则是在主程序中直接创建一个ov::Tensor变量，然后传入infer_request中。这两种创建方法有什么不同效果吗？
#ifndef AUTO_AIM__YOLOV5_HPP
#define AUTO_AIM__YOLOV5_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "tasks/auto_aim/armor.hpp"
// #include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim {

class YOLOV5 : public YOLOBase {
   public:
    YOLOV5(const std::string& config_path, bool debug);

    std::list<Armor> detect(const cv::Mat& raw_img, int frame_count) override;
    // 主程序，执行推理并获取输出层

    std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                 int frame_count) override;
    // 保留接口，可能用于之后的异步推理

   private:
    std::string device_;      ///< 使用 CPU 还是 GPU 推理
    std::string model_path_;  ///< 模型路径
    std::string save_path_;   ///< 照片保存路径，用于之后训练模型
    bool debug_;              ///< 是否绘图
    bool use_roi_;            ///< 是否启用ROI裁剪图片
    bool use_traditional_;    ///< 是否使用传统方法校正
    // Detector detector_;
    // friend class MultiThreadDetector;

    // OpenVINO 相关
    ov::Core core_;  // OpenVINO Runtime Core对象
    ov::CompiledModel compiled_model_;

    // ROI 相关
    cv::Rect roi_;
    cv::Point2f offset_;

    std::list<Armor> parse(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                           int frame_count);  // 读取output中的数据并返回识别到的装甲板

    double sigmoid(double x);                                 // 将置信度归一化
    void save(const cv::Mat& img, const Armor& armor) const;  // 保存识别错误图片

    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_HPP
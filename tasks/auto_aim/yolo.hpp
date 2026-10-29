#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim {
/* YOLO 抽象基类，用于之后被 yolov5、yolov8 或 yolov11 类继承 */
class YOLOBase {
   public:
    virtual std::list<Armor> detect(const cv::Mat& img, int frame_count) = 0;

    virtual std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                         int frame_count) = 0;
};

/* YOLO 主类，使用指针调用选定的 YOLO 版本*/
class YOLO {
   public:
    YOLO(const std::string& config_path, bool debug = false);

    std::list<Armor> detect(const cv::Mat& img, int frame_count = -1);

    std::list<Armor> postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img,
                                 int frame_count);

   private:
    std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP

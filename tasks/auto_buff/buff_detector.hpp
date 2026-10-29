#ifndef AUTO_BUFF__DETECTOR_HPP
#define AUTO_BUFF__DETECTOR_HPP

#include <optional>
#include <vector>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"
#include "yolo11_buff.hpp"

const int LOSE_MAX = 60;  // 最大目标丢失帧数，超出这个帧视为彻底丢失能量机关目标
const int PREDICT_LOSE_THRESH = 20;  // 预测跟踪模式的阈值帧数，小于这个阈值使用预测跟踪模式

namespace auto_buff {
/* Buff_Detector类 */
class Buff_Detector {
   public:
    Buff_Detector(const std::string& config_path, const std::string& model_key);
    // void setEnergyType(EnergyType type) { energy_type_ = type; }

    std::optional<PowerRune> detect(cv::Mat& bgr_img);

   private:
    // EnergyType energy_type_;
    // BuffColor energy_color_; // 需要打的颜色
    YOLO11_BUFF yolo_model_;

    // 跟踪状态量
    TrackStatus status_;
    int lose_;

    // 预测跟踪相关
    std::chrono::steady_clock::time_point timestamp_;
    std::optional<PowerRune> last_powerrune_;
    PowerRune predicted_powerrune_;
    bool prediction_valid_;

    std::optional<PowerRune> handle_target_lose();  // 目标丢失时的各种处理
}

}  // namespace auto_buff

#endif  // AUTO_BUFF__DETECTOR_HPP
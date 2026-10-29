#ifndef AUTO_BUFF__BUFF_TYPE_HPP
#define AUTO_BUFF__BUFF_TYPE_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>

#include "tools/math_tools.hpp"

namespace auto_buff {
// 枚举类
enum class EnergyType { SMALL, BIG };
enum class BuffColor { RED, BULE };
enum class TrackStatus { TRACK, TEM_LOSE, LOSE, PREDICT_TRACK };
enum class FanBladeStatus { TARGET, UNLIGHT, LIGHT };

/* 扇叶类，存储能量机关上单个扇叶的信息 */
class FanBlade {
   public:
    cv::Point2f center_;
    std::vector<cv::Point2f> kpts_;
    FanBladeStatus f_b_status_;
    double angle_;
    explicit FanBlade(const std::vector<cv::Point2f>& kpts);
};

/* 能量机关类 */
class PowerRune {
   public:
    // 能量机关构成成分
    cv::Point2f r_center_;
    std::vector<FanBlade> fanblades_;
    int light_num_;  // 亮起的扇叶数（其实就是传入的fanblades.size()）

    explicit PowerRune(std::vector<FanBlade>& fanblades, const cv::Point2f r_center,
                       std::optional<PowerRune> last_powerrune);
    // FanBlade & target_fanblade(){ return ;}
   private:
    bool unsolvable_ = false;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__BUFF_TYPE_HPP
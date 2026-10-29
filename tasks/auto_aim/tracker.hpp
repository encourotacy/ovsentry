#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <chrono>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim {

class Tracker {
   public:
    Tracker(const std::string& config_path, Solver& solver);

    std::string state() const { return state_; };

    void filter(std::list<Armor>& armors, cv::Point2f img_center,
                size_t invincibility = static_cast<size_t>(0));  // 过滤装甲板

    std::list<Target> track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);

   private:
    Solver& solver_;
    Color enemy_color_;

    std::chrono::steady_clock::time_point last_timestamp_;
    std::string state_;  // 当前跟踪状态
    int detect_count_;   // 观察到目标的帧数
    int lost_count_;     // 丢失目标的帧数
    Target target_;      // 目标

    void state_machine(bool found);
    bool set_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
    bool update_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
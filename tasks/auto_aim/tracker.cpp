#include "tracker.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace {
constexpr int MIN_DETECT_COUNT = 5;     // 连续检测到目标的最小次数
constexpr int MAX_LOST_COUNT = 15;      // 连续丢失目标的最大次数
constexpr int OUTPOST_LOST_COUNT = 75;  // 连续丢失前哨站的最大次数
}  // namespace

namespace auto_aim {
//==== 初始化 ===
Tracker::Tracker(const std::string &config_path, Solver &solver)
    : solver_{solver}, state_{"lost"}, last_timestamp_(std::chrono::steady_clock::now()) {
    auto yaml = tools::load(config_path);
    enemy_color_ =
        (tools::read<std::string>(yaml, "enemy_color") == "red") ? Color::RED : Color::BLUE;
}
//=== 主程序 ===
void Tracker::filter(std::list<Armor> &armors, cv::Point2f img_center, size_t invincibility) {
    // 过滤掉非我方装甲板
    armors.remove_if([&](const Armor &a) { return a.color_ != enemy_color_; });

    // 过滤无敌单位的装甲板
    if (invincibility) {
        for (int i = 0; i < 5; i++) {
            if (invincibility % 2) {
                auto filterd_armor = ArmorName(i);
                armors.remove_if([&](const Armor &a) { return a.name_ == filterd_armor; });
            }
            invincibility = invincibility >> 1;
        }
    }

    // 优先选择靠近图像中心的装甲板
    armors.sort([&](const Armor &a, const Armor &b) {
        auto distance_1 = cv::norm(a.center_ - img_center);
        auto distance_2 = cv::norm(b.center_ - img_center);
        return distance_1 < distance_2;
    });

    // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
    armors.sort([](const Armor &a, const Armor &b) { return a.priority_ < b.priority_; });
}

std::list<Target> Tracker::track(std::list<Armor> &armors,
                                 std::chrono::steady_clock::time_point t) {
    //---检查时间间隔 ---
    auto dt = tools::delta_time(last_timestamp_, t);
    last_timestamp_ = t;

    // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1) {
        tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
        if (target_.name != ArmorName::OUTPOST) {
            state_ = "lost";
        }
    }

    //--- 根据状态选择不同的跟踪策略 ---
    bool found;
    if (state_ == "lost") {
        found = set_target(armors, t);
    } else {
        found = update_target(armors, t);
    }

    state_machine(found);

    if (state_ != "lost" && target_.diverged()) {
        tools::logger()->debug("[Tracker] Target diverged!");
        state_ = "lost";
        return {};
    }

    if (state_ != "lost" && target_.ekf().nis_fail()) {
        tools::logger()->debug("[Target] Bad Converge Found!");
        state_ = "lost";
        return {};
    }
    if (state_ == "lost") return {};

    std::list<Target> targets = {target_};
    return targets;
}

//=== 目标创建与更新 ===
void Tracker::state_machine(bool found) {
    if (state_ == "lost") {
        if (!found) return;
        state_ = "detecting";
        detect_count_ = 1;
    } else if (state_ == "detecting") {
        if (found) {
            detect_count_++;
            if (detect_count_ >= MIN_DETECT_COUNT) state_ = "tracking";
        } else {
            detect_count_ = 0;
            state_ = "lost";
        }
    } else if (state_ == "tracking") {
        if (found) return;

        lost_count_ = 1;
        state_ = "temp_lost";
    } else if (state_ == "switching") {
        if (found) {
            state_ = "detecting";
        } else {
            lost_count_++;
            if (lost_count_ > 200) state_ = "lost";
        }
    } else if (state_ == "temp_lost") {
        if (found) {
            state_ = "tracking";
        } else {
            lost_count_++;
            if (target_.name == ArmorName::OUTPOST) {
                if (lost_count_ > OUTPOST_LOST_COUNT) state_ = "lost";
            } else {
                if (lost_count_ > MAX_LOST_COUNT) state_ = "lost";
            }
        }
    }
}

bool Tracker::set_target(std::list<Armor> &armors, std::chrono::steady_clock::time_point t) {
    if (armors.empty()) return false;

    auto &armor = armors.front();
    solver_.solve(armor);

    auto is_balance = (armor.type_ == ArmorType::BIG) &&
                      (armor.name_ == ArmorName::THREE || armor.name_ == ArmorName::FOUR);

    if (is_balance) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 2, P0_dig);
    } else if (armor.name_ == ArmorName::OUTPOST) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 25, 81, 0.4, 100, 1e-4, 0, 0}};
        target_ = Target(armor, t, 0.2765, 3, P0_dig);
    } else if (armor.name_ == ArmorName::BASE) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
        target_ = Target(armor, t, 0.3205, 3, P0_dig);
    } else {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 4, P0_dig);
    }

    return true;
}

bool Tracker::update_target(std::list<Armor> &armors, std::chrono::steady_clock::time_point t) {
    target_.predict(t);

    bool found = false;
    for (auto &armor : armors) {
        if (armor.name_ != target_.name) continue;
        found = true;
        solver_.solve(armor);
        target_.update(armor);
    }
    if (!found) return false;

    return true;
}

}  // namespace auto_aim
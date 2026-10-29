#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "io/command.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim {
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

class Planner {
   public:
    Eigen::Vector4d aim_xyza;  // 瞄准装甲板位置，调试程序画图用
    Planner(const std::string& config_path, bool debug = false);

    io::MPCCommand plan(Target target, double bullet_speed);
    io::MPCCommand plan(std::optional<Target> target, double bullet_speed);

   private:
    // 参数
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double low_speed_delay_time_;
    double high_speed_delay_time_;
    double decision_speed_;
    double resistance_k_;
    double last_yaw_ = 0;
    double traj_g_;

    bool debug_;

    std::vector<double> planner_judge_distance_;
    std::vector<double> planner_fire_thresh_;
    bool is_multiple_thresh_ = true;  // 是否开启多距离不同阈值

    // 强制开火参数和变量
    bool high_spin_force_active_planner_ = false;
    bool high_spin_force_enabled_planner_ = false;
    double high_spin_force_enter_speed_planner_;
    double high_spin_force_exit_speed_planner_;
    double plus_k_;

    TinySolver* yaw_solver_;
    TinySolver* pitch_solver_;

    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    double get_thresh(double distance, double angular_speed);
    Eigen::Matrix<double, 2, 1> aim(const Target& target, double bullet_speed, bool debug = false); // 根据瞄准点计算MPC参考轨迹最终应该到达的位置
    Trajectory get_trajectory(Target& target, double yaw0, double bullet_speed); // 计算MPC参考轨迹
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
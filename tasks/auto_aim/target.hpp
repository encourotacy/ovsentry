#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim {

class Target {
   public:
    ArmorName name;
    Target() = default;
    Target(const Armor& armor, std::chrono::steady_clock::time_point t, double radius,
           int armor_num, Eigen::VectorXd P0_dig);

    // 预测功能
    void predict(std::chrono::steady_clock::time_point t);
    void predict(double dt);          // 进行先验预测
    void update(const Armor& armor);  // 根据观测值更新预测

    // 查询数据
    bool diverged() const;  // 是否发散，判断标准为 r 或 l 是否出现异常数据
    bool convergened();  // 检查是否收敛并更新标识，判断标准为观测时间是否足够
    Eigen::VectorXd ekf_x() const;                   // 返回旋转中心的状态向量
    const tools::ExtendedKalmanFilter& ekf() const;  // 返回EKF，也可以查询状态向量
    std::vector<Eigen::Vector4d> armor_xyza_list() const;  // 返回由旋转中心推测出的装甲板世界坐标

    // debug
    // 是否打开一些对于前哨站的优化，如果试验有效的话之后会删去
    bool outpost_Q_optimize_ = false;
    bool outpost_R_optimize_ = false;

   private:
    tools::ExtendedKalmanFilter ekf_;
    std::chrono::steady_clock::time_point t_;  // 目标的最后更新时间

    int armor_num_;
    int update_count_;

    int match_id(const Armor& armor);  // 匹配以得到输入装甲板的id
    Eigen::Vector3d speculate_armor_xyz(const Eigen::VectorXd& x,
                                        int id) const;  // 计算出各个装甲板中心的坐标（考虑长短轴）
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd& x,
                               int id) const;  // 计算 z = h(x) 的雅可比矩阵
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
#ifndef AUTO_ATM__SOLVER_HPP
#define AUTO_ATM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim {

class Solver {
   public:
    explicit Solver(std::string config_path);              // 构造函数
    void set_R_gimbal2world(const Eigen::Quaterniond& q);  // 传入当前的四元数以获取 R_gimbal2world

    void solve(Armor& armor) const;  // 主程序，计算armor的各种坐标
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d& xyz_in_world, double yaw,
                                             ArmorName name) const;
    // 重投影，输入装甲板中心，根据之前的解算结果重新计算装甲板角点坐标\

   private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;

    void optimize_yaw(Armor& armor) const;  // 修正PnP解算的yaw角
};

}  // namespace auto_aim

#endif  // AUTO_ATM__SOLVER_HPP
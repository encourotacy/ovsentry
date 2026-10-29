#ifndef AUTO_BUFF__BUFF_SOLVER
#define AUTO_BUFF__BUFF_SOLVER

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff {
/*  */
class Solver {
   public:
    explicit Solver(const std::string& config_path);
    void set_R_gimbal2world(const Eigen::Quaterniond& q);
    void solve(std::optional<PowerRune>& ps);

    // 调试用
    Eigen::Matrix3d R_gimbal2world() const;
    cv::Point2f point_buff2pixel(cv::Point3f x);

    //---旧版接口---

   private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;

    cv::Vec3d rvec_, tvec_;

    const std::vector<cv::Point3f> OBJECT_POINTS = {
        cv::Point3f(0, 0, 825e-3), cv::Point3f(0, 125e-3, 700e-3),
        cv::Point3f(0, 0, 575e-3), cv::Point3f(0, -125e-3, 700e-3),
        cv::Point3f(0, 0, 700e-3), cv::Point3f(0, 0, 180e-3)};  // 单位：m
    // 扇叶（竖直向上时）关键点在以能量机关中心为原点的xyz坐标
    // 但是解算时将其视为平面，因此x坐标总是0
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__BUFF_SOLVER
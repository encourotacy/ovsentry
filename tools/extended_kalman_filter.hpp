#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools {

class ExtendedKalmanFilter {
   public:
    Eigen::VectorXd x;  ///< 状态向量
    Eigen::MatrixXd P;  ///< 预测协方差矩阵

    ExtendedKalmanFilter() = default;
    ExtendedKalmanFilter(
        const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
            [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a + b; },
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)>
            z_subtract = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a - b; },
        bool debug = true);

    // 先验估计
    Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
                            std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

    // 后验估计
    Eigen::VectorXd update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H,
                           const Eigen::MatrixXd &R,
                           std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h);

    bool nis_fail() const { return nis_fail_; }

    std::map<std::string, double> data;  //卡方检验数据

   private:
    std::deque<int> recent_nis_failures{0};

    bool debug_;
    bool nis_fail_;
    Eigen::MatrixXd I;  // 单位阵，用于之后的计算
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
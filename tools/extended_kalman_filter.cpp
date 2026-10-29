#include "extended_kalman_filter.hpp"

#include <numeric>

namespace {

constexpr double NIS_THRESH = 0.711;  // 卡方检验阈值（自由度=4，取置信水平95%）
constexpr size_t WINDOW_SIZE = 100;   // 检验队列长度

}  // namespace

namespace tools {

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
    bool debug)
    : x(x0),
      P(P0),
      debug_{debug},
      I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())),
      x_add(x_add),
      z_subtract(z_subtract) {
    data["innovation_yaw"] = 0.0;
    data["innovation_pitch"] = 0.0;
    data["innovation_distance"] = 0.0;
    data["innovation_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
    const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f) {
    P = F * P * F.transpose() + Q;  // 计算先验误差协方差矩阵
    x = f(x);                       // 先验状态估计
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h) {
    auto hx = h(x);
    // NIS 检验
    Eigen::VectorXd innovation = z_subtract(z, hx);
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    double nis = innovation.transpose() * S.inverse() * innovation;

    data["nis_fail"] = (nis > NIS_THRESH) ? 1.0 : 0.0;
    recent_nis_failures.push_back(nis > NIS_THRESH ? 1 : 0);

    if (recent_nis_failures.size() > WINDOW_SIZE) {
        recent_nis_failures.pop_front();
    }

    int recent_failures =
        std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

    nis_fail_ = (recent_failures >= (0.4 * WINDOW_SIZE));

    if (debug_) {
        data["innovation_yaw"] = innovation[0];
        data["innovation_pitch"] = innovation[1];
        data["innovation_distance"] = innovation[2];
        data["innovation_angle"] = innovation[3];
        data["nis"] = nis;
        data["recent_nis_failures_rate"] = recent_rate;
    }

    Eigen::MatrixXd K =
        P * H.transpose() * (H * P * H.transpose() + R).inverse();  // 计算卡尔曼增益

    // Stable Compution of the Posterior Covariance
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    // 这里使用未简化的形式计算是刻意而为之的，旨在让数值更加稳定。
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();  // 更新协方差矩阵
    x = x_add(x, K * z_subtract(z, hx));                                    // 更新状态估计

    return x;
}

}  // namespace tools
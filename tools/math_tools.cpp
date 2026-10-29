#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI

namespace tools {
//=== 直角坐标系与球坐标系互化 ===
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];
    auto yaw = std::atan2(y, x);
    auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
    auto distance = std::sqrt(x * x + y * y + z * z);
    return {yaw, pitch, distance};
}

Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd) {
    auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    auto x = distance * std::cos(pitch) * std::cos(yaw);
    auto y = distance * std::cos(pitch) * std::sin(yaw);
    auto z = distance * std::sin(pitch);
    return {x, y, z};
}

Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];

    auto dyaw_dx = -y / (x * x + y * y);
    auto dyaw_dy = x / (x * x + y * y);
    auto dyaw_dz = 0.0;

    auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

    // clang-format off
    Eigen::MatrixXd J{
        {dyaw_dx, dyaw_dy, dyaw_dz},
        {dpitch_dx, dpitch_dy, dpitch_dz},
        {ddistance_dx, ddistance_dy, ddistance_dz}
    };
    // clang-format on

    return J;
}

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d& ypd) {
    auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);

    auto dx_dyaw = distance * cos_pitch * -sin_yaw;
    auto dy_dyaw = distance * cos_pitch * cos_yaw;
    auto dz_dyaw = 0.0;

    auto dx_dpitch = distance * -sin_pitch * cos_yaw;
    auto dy_dpitch = distance * -sin_pitch * sin_yaw;
    auto dz_dpitch = distance * cos_pitch;

    auto dx_ddistance = cos_pitch * cos_yaw;
    auto dy_ddistance = cos_pitch * sin_yaw;
    auto dz_ddistance = sin_pitch;

    // clang-format off
    Eigen::MatrixXd J{
        {dx_dyaw, dx_dpitch, dx_ddistance},
        {dy_dyaw, dy_dpitch, dy_ddistance},
        {dz_dyaw, dz_dpitch, dz_ddistance}
    };
    // clang-format on

    return J;
}
//=== 四元数转欧拉角 ===
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic) {
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2 = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    auto safe = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic) {
    Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

//=== 小工具 ===
double rad2deg(double rad) { return rad * 180.0 / CV_PI; }
double deg2rad(double deg) { return deg * CV_PI / 180.0; }

double limit_rad(double angle) {
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle < -CV_PI) angle += 2 * CV_PI;
    return angle;
}

double delta_time(const std::chrono::steady_clock::time_point& start,
                  const std::chrono::steady_clock::time_point& end) {
    std::chrono::duration<double> elapsed = end - start;
    return elapsed.count();
}

Eigen::Quaterniond ypr_rad2q(double yaw_deg, double pitch_deg, double roll_deg) {
    return Eigen::AngleAxisd(deg2rad(yaw_deg), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(deg2rad(pitch_deg), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(deg2rad(roll_deg), Eigen::Vector3d::UnitX());
}

}  // namespace tools
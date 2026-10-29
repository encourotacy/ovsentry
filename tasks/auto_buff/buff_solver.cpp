#include "buff_solver.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/yaml.hpp"

namespace auto_buff {
//=== 初始化 ===
Solver::Solver(const std::string& config_path) {
    auto yaml = tools::load(config_path);

    auto R_gimbal2imubody_data = tools::read<std::vector<double>>(yaml, "R_gimbal2imu");
    auto R_camera2gimbal_data = tools::read<std::vector<double>>(yaml, "R_camera2gimbal");
    auto t_camera2gimbal_data = tools::read<std::vector<double>>(yaml, "t_camera2gimbal");
    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
    R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeffs, distort_coeffs_);
}
void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//=== 主程序 ===
void Solver::solve(std::optional<PowerRune>& ps) {
    if (!ps.has_value()) return;
    PowerRune& p = ps.value();

    //--- PnP解算 ---
    std::vector<cv::Point2f> image_points = p.target_fanblade().kpts;
    std::vector<cv::Point2f> image_points_fourth(image_points.begin(), image_points.begin() + 4);
    std::vector<cv::Point3f> OBJECT_POINTS_FOURTH(OBJECT_POINTS.begin(), OBJECT_POINTS.begin() + 4);
    cv::solvePnP(
        OBJECT_POINTS_FOURTH,  // 1. 世界坐标系中的3D点（单位：米）
        image_points_fourth,   // 2. 对应的图像像素坐标（单位：像素）
        camera_matrix_,        // 3. 相机内参矩阵 (3x3)：[fx, 0, cx; 0, fy, cy; 0, 0, 1]
        distort_coeffs_,       // 4. 畸变系数 (k1, k2, p1, p2, k3)
        rvec_,                 // 5. 输出的旋转向量 (3x1)
        tvec_,                 // 6. 输出的平移向量 (3x1)
        false,  // 7. useExtrinsicGuess：是否使用上一帧的rvec/tvec作为初始值，false表示不用
        cv::SOLVEPNP_IPPE  // 8. 求解方法：IPPE（Infinitesimal Plane-Based Pose Estimation）
    );

    Eigen::Vector3d t_buff2camera;
    cv::cv2eigen(tvec_, t_buff2camera);
    cv::Mat rmat;                // 这不是一个图像，cv::Mat也可以存储数学矩阵。
    cv::Rodrigues(rvec_, rmat);  // 罗德里格斯公式，将旋转向量转化为旋转矩阵
    Eigen::Matrix3d R_buff2camera;
    cv::cv2eigen(rmat, R_buff2camera);

    //--- 坐标变换 ---
    Eigen::Vector3d blade_xyz_in_buff{{0, 0, 700e-3}};

    // buff -> camera
    Eigen::Vector3d xyz_in_camera = t_buff2camera;
    Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

    // camera -> gimbal
    Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
    Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

    // gimbal -> world
    Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;

    p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
    p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

    p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
    p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);

    p.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
}

//=== 读取 ===
Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

//=== 调试用 ===
cv::Point2f Solver::point_buff2pixel(cv::Point3f x) {
    // buff坐标系(单位:m)到像素坐标系
    std::vector<cv::Point3d> world_points;
    std::vector<cv::Point2d> image_points;
    world_points.push_back(x);
    cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
    return image_points.back();
}

}  // namespace auto_buff
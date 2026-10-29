#include "solver.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace {
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m
constexpr double BASE_ARMOR_WIDTH = 129e-3;   // m
constexpr double SEARCH_RANGE = 140;          // degree, optimize_yaw中的搜索角度

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{{0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{{0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> BASE_ARMOR_POINTS{{0, BASE_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                 {0, -BASE_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                 {0, -BASE_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                 {0, BASE_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

}  // namespace

namespace auto_aim {
//=== 初始化 ===
Solver::Solver(std::string config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity()) {
    auto yaml = tools::load(config_path);

    auto R_gimbal2imubody_data = tools::read<std::vector<double>>(yaml, "R_gimbal2imubody");
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
    // R_imubody2imuabs是 IMU 本体到 IMU 绝对坐标系（也就是我们定义的世界坐标系）的旋转。
}

//=== 主程序 ===
void Solver::solve(Armor& armor) const {
    cv::Vec3d rvec, tvec;
    cv::solvePnP(
        SMALL_ARMOR_POINTS,  // 1. 世界坐标系中的3D点（单位：米）
        armor.points_,       // 2. 对应的图像像素坐标（单位：像素）
        camera_matrix_,      // 3. 相机内参矩阵 (3x3)：[fx, 0, cx; 0, fy, cy; 0, 0, 1]
        distort_coeffs_,     // 4. 畸变系数 (k1, k2, p1, p2, k3)
        rvec,                // 5. 输出的旋转向量 (3x1)
        tvec,                // 6. 输出的平移向量 (3x1)
        false,  // 7. useExtrinsicGuess：是否使用上一帧的rvec/tvec作为初始值，false表示不用
        cv::SOLVEPNP_IPPE  // 8. 求解方法：IPPE（Infinitesimal Plane-Based Pose Estimation）
    );

    Eigen::Vector3d t_armor2camera;
    cv::cv2eigen(tvec, t_armor2camera);
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);  // 罗德里格斯公式，将旋转向量转化为旋转矩阵
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;

    armor.xyz_in_gimbal_ = R_camera2gimbal_ * t_armor2camera + t_camera2gimbal_;
    armor.xyz_in_world_ = R_gimbal2world_ * armor.xyz_in_gimbal_;
    armor.ypr_in_gimbal_ = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world_ = tools::eulers(R_armor2world, 2, 1, 0);
    armor.ypd_in_world_ = tools::xyz2ypd(armor.xyz_in_world_);

    optimize_yaw(armor);
}

void Solver::optimize_yaw(Armor& armor) const {
    Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

    auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

    auto min_error = 1e10;
    auto best_yaw = armor.ypr_in_world_[0];

    for (int i = 0; i < SEARCH_RANGE; i++) {
        double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
        auto image_points = reproject_armor(armor.xyz_in_world_, yaw, armor.name_);
        auto error = 0.0;
        for (int i = 0; i < 4; i++) {
            error += cv::norm(armor.points_[i] - image_points[i]);
        }
        if (error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }

    armor.yaw_raw = armor.ypr_in_world_[0];
    armor.ypr_in_world_[0] = best_yaw;
}

//=== 工具方法 ===
std::vector<cv::Point2f> Solver::reproject_armor(const Eigen::Vector3d& xyz_in_world, double yaw,
                                                 ArmorName name) const {
    // 将欧拉角转为旋转矩阵
    auto pitch = (name == ArmorName::OUTPOST) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    const Eigen::Matrix3d R_armor2world{{cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
                                        {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
                                        {-sin_pitch, 0, cos_pitch}};

    // 重投影
    const Eigen::Vector3d& t_armor2world = xyz_in_world;
    Eigen::Matrix3d R_armor2camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera =
        R_camera2gimbal_.transpose() *
        (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(SMALL_ARMOR_POINTS, rvec, tvec, camera_matrix_, distort_coeffs_,
                      image_points);
    return image_points;
}

}  // namespace auto_aim
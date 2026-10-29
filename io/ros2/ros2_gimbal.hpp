#ifndef IO__ROS2_GIMBAL_HPP
#define IO__ROS2_GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "rm_interfaces/msg/gimbal.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

struct ROS2GimbalState {
    double small_yaw;     ///< 云台小 yaw，单位为 rad
    double yaw_vel;       ///< 云台 yaw 角速度，单位为 rad/s
    double pitch;         ///< 云台 pitch 角，单位为 rad
    double pitch_vel;     ///< 云台 pitch 角速度，单位为 rad/s
    double bullet_speed;  ///< 弹速，单位为 m/s
    double big_yaw;       ///< 底盘 yaw 角（大 yaw）
};

class ROS2Gimbal {
   public:
    explicit ROS2Gimbal(const std::string& config_path);
    ~ROS2Gimbal();

    // 数据访问
    Eigen::Quaterniond q_at(
        std::chrono::steady_clock::time_point image_timestamp); ///< 返回这个时间的IMU四元数
    double big_yaw_at_image(
        std::chrono::steady_clock::time_point image_timestamp); ///< 返回拍照时刻的大 yaw，单位 rad
    double bullet_speed() const;                                ///< 返回弹速
    ROS2GimbalState state() const;

    // 使用 ROS2 话题发布指令
    void send(const io::Command& c);
    void send(const io::MPCCommand& c);
    void send(bool control, bool fire, double big_yaw, double small_yaw, double pitch,
              double yaw_vel, double pitch_vel, double yaw_acc, double pitch_acc, uint8_t armor_id,
              double vx, double vy, double distance);  ///< 手动组织数据

   private:
    struct IMUData {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
        double big_yaw_rad = 0.0;
    };
    tools::ThreadSafeQueue<IMUData> queue_{1000};

    std::chrono::microseconds imu_query_offset_{-1000};
    std::atomic<bool> self_initialized_{false};  ///< rclcpp是否初始化
    mutable std::mutex mtx_;                     ///< 互斥锁

    std::string node_name_;
    std::string status_topic_;
    std::string cmd_topic_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>> cmd_publisher_;
    std::shared_ptr<rclcpp::Subscription<rm_interfaces::msg::Gimbal>> status_subscription_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;

    IMUData data_ahead_;
    IMUData data_behind_;
    IMUData data_prev_;
    IMUData latest_data_;
    bool has_prev_{false};

    // 最近一帧 /serial/gimbal_status，数值来自电控，由导航转发
    std::atomic<bool> is_receiving_data_{false};
    double small_yaw_ = 0.0;
    double pitch_ = 0.0;
    double yaw_vel_ = 0.0;
    double pitch_vel_ = 0.0;
    double bullet_speed_;
    double big_yaw_ = 0.0;

    // 初始化
    void status_callback(const rm_interfaces::msg::Gimbal::SharedPtr msg);  // 数据接收
    void read_config(const std::string& config_path);                       // 读取参数
};

}  // namespace io

#endif  // IO__ROS2_GIMBAL_HPP
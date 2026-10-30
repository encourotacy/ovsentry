#include "ros2_gimbal.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace {}  // namespace

namespace io {
//=== 初始化 ===
void ROS2Gimbal::status_callback(const rm_interfaces::msg::Gimbal::SharedPtr msg) {
    try {
        const double pitch_deg = msg->pitch;
        const double roll_deg = msg->roll;
        const double yaw_deg = msg->yaw;
        const double big_yaw_deg = msg->big_yaw;
        const double yaw_vel_deg = msg->small_yaw_speed;
        const double pitch_vel_deg = msg->pitch_speed;
        const double bullet_speed = msg->bullet_speed;

        Eigen::Quaterniond q(msg->w, msg->x, msg->y, msg->z);

        const double norm = q.norm();
        if (!(std::isfinite(norm) && std::abs(norm - 1.0) < 1e-2)) {
            q = tools::ypr_rad2q(yaw_deg, pitch_deg, roll_deg);
        } else {
            q.normalize();
        }

        const auto now = std::chrono::steady_clock::now();
        queue_.push({q, now, tools::deg2rad(big_yaw_deg)});

        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_data_ = {q, now};
            is_receiving_data_ = true;
            small_yaw_ = tools::deg2rad(yaw_deg);
            pitch_ = tools::deg2rad(pitch_deg);
            yaw_vel_ = tools::deg2rad(yaw_vel_deg);
            pitch_vel_ = tools::deg2rad(pitch_vel_deg);
            big_yaw_ = tools::deg2rad(big_yaw_deg);
            if (bullet_speed > 10 && bullet_speed < 27) bullet_speed_ = bullet_speed;
        }
    } catch (const std::exception& e) {
        tools::logger()->warn("[ROS2Gimbal] Failed to parse status message: {}", e.what());
    }
}

void ROS2Gimbal::read_config(const std::string& config_path) {
    //--- 读取参数 ---
    auto yaml = tools::load(config_path);
    bullet_speed_ = tools::read<double>(yaml, "default_bullet_speed", 23.0);  // 读取默认弹速

    double offset_s = tools::read<double>(yaml["timing"], "offset", -0.001);
    imu_query_offset_ =
        std::chrono::microseconds(static_cast<int64_t>(std::llround(offset_s * 1e6)));

    //--- 读取云台桥话题名称 ---
    auto bridge = yaml["ros2_gimbal"];

    node_name_ = tools::read<std::string>(bridge, "node_name");
    status_topic_ = tools::read<std::string>(bridge["subscribe"], "status_topic");
    cmd_topic_ = tools::read<std::string>(bridge["publish"], "cmd_topic");
}

ROS2Gimbal::ROS2Gimbal(const std::string& config_path) {
    //--- 读取参数 ---
    read_config(config_path);

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);  // rclcpp 初始化
        self_initialized_ = true;
    }

    //--- 创建ROS2通信用变量 ---
    node_ = std::make_shared<rclcpp::Node>(node_name_);
    cmd_publisher_ =
        node_->create_publisher<rm_interfaces::msg::GimbalCmd>(cmd_topic_, rclcpp::SensorDataQoS());
    status_subscription_ = node_->create_subscription<rm_interfaces::msg::Gimbal>(
        status_topic_, rclcpp::SensorDataQoS(),
        [this](const rm_interfaces::msg::Gimbal::SharedPtr msg) { this->status_callback(msg); });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    //--- 读取首批IMU数据 ---
    tools::logger()->info("[ROS2Gimbal] Waiting for IMU...");
    if (queue_.pop(data_ahead_, std::chrono::milliseconds(2000)) &&
        queue_.pop(data_behind_, std::chrono::milliseconds(2000))) {
        data_prev_ = data_ahead_;
        has_prev_ = true;
        tools::logger()->info("[ROS2Gimbal] First IMU pair received.");
    } else {
        const auto now = std::chrono::steady_clock::now();
        data_prev_ = {Eigen::Quaterniond::Identity(), now};
        data_ahead_ = data_prev_;
        data_behind_ = data_prev_;
        tools::logger()->warn("[ROS2Gimbal] IMU startup timeout, using latest-available fallback.");
    }
}

ROS2Gimbal::~ROS2Gimbal() {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    if (executor_ && node_) executor_->remove_node(node_);
    if (self_initialized_ && rclcpp::ok()) rclcpp::shutdown();
}

//=== 数据访问 ===
Eigen::Quaterniond ROS2Gimbal::q_at(std::chrono::steady_clock::time_point image_timestamp) {
    // 由于照片时刻和imu传入数据的时刻不一定相同，需要计算出拍照时imu的四元数。
    if (!is_receiving_data_) return Eigen::Quaterniond::Identity();  // 没收到数据则返回无

    auto timestamp = image_timestamp + imu_query_offset_;

    auto interpolate = [&](const IMUData& a, const IMUData& b) {
        // 球面线性插值计算两 IMUData 之间最可能的四元数
        auto q_a = a.q.normalized();
        auto q_b = b.q.normalized();
        const double dt = tools::delta_time(a.timestamp, b.timestamp);
        if (dt <= 1e-6) return q_b;
        const double k = std::clamp(tools::delta_time(a.timestamp, timestamp) / dt, 0.0, 1.0);
        return q_a.slerp(k, q_b).normalized();
    };

    if (timestamp <= data_ahead_.timestamp) {
        if (has_prev_ && timestamp >= data_prev_.timestamp) {
            return interpolate(data_prev_, data_ahead_);
        }
        return data_ahead_.q.normalized();
    }

    while (data_behind_.timestamp < timestamp) {
        // "滑动窗口"移动
        data_prev_ = data_ahead_;
        data_ahead_ = data_behind_;
        has_prev_ = true;
        if (!queue_.pop(data_behind_, std::chrono::milliseconds(2))) {  // 从队列中读取data_behind_
            // 读取 data_behind_ 失败的处理
            std::lock_guard<std::mutex> lk(mtx_);
            if (is_receiving_data_ && latest_data_.timestamp > data_ahead_.timestamp) {
                return interpolate(data_ahead_, latest_data_);
            }
            return data_ahead_.q.normalized();
        }
    }

    // 若正常退出while，则说明 timestamp 在 data_ahead_ 和 data_behind_ 之间
    return interpolate(data_ahead_, data_behind_);
}

// 目前这里存疑，还没修改好
double ROS2Gimbal::big_yaw_at_image(std::chrono::steady_clock::time_point image_timestamp) {
    if (!is_receiving_data_) {
        std::lock_guard<std::mutex> lk(mtx_);
        return big_yaw_;
    }

    const auto timestamp = image_timestamp + imu_query_offset_;
    auto interp = [](const IMUData& a, const IMUData& b,
                     std::chrono::steady_clock::time_point t) {
        const double dt = tools::delta_time(a.timestamp, b.timestamp);
        if (dt <= 1e-6) return b.big_yaw_rad;
        const double k = std::clamp(tools::delta_time(a.timestamp, t) / dt, 0.0, 1.0);
        return a.big_yaw_rad + tools::limit_rad(b.big_yaw_rad - a.big_yaw_rad) * k;
    };

    if (timestamp <= data_ahead_.timestamp) {
        if (has_prev_ && timestamp >= data_prev_.timestamp) {
            return interp(data_prev_, data_ahead_, timestamp);
        }
        return data_ahead_.big_yaw_rad;
    }

    while (data_behind_.timestamp < timestamp) {
        data_prev_ = data_ahead_;
        data_ahead_ = data_behind_;
        has_prev_ = true;
        IMUData next;
        if (!queue_.pop(next, std::chrono::milliseconds(0))) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (is_receiving_data_ && latest_data_.timestamp > data_ahead_.timestamp) {
                IMUData latest = latest_data_;
                latest.big_yaw_rad = big_yaw_;
                return interp(data_ahead_, latest, timestamp);
            }
            return data_ahead_.big_yaw_rad;
        }
        data_behind_ = next;
    }

    return interp(data_ahead_, data_behind_, timestamp);
}

double ROS2Gimbal::bullet_speed() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bullet_speed_;
}

ROS2GimbalState ROS2Gimbal::state() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return {small_yaw_, yaw_vel_, pitch_, pitch_vel_, bullet_speed_, big_yaw_};
}

//=== 发送 ===
void ROS2Gimbal::send(const io::Command& c) {
    send(c.control, c.shoot, c.big_yaw, c.small_yaw, c.pitch, 0.0, 0.0, 0.0, 0.0, c.armor_name_num,
         0.0, 0.0, 0.0);
}

void ROS2Gimbal::send(const io::MPCCommand& c) {
    send(c.control, c.shoot, c.big_yaw, c.small_yaw, c.pitch, c.yaw_vel, c.pitch_vel, c.yaw_acc,
         c.pitch_acc, c.armor_name_num, c.vx, c.vy, c.distance);
}

void ROS2Gimbal::send(bool control, bool fire, double big_yaw, double small_yaw, double pitch,
                      double yaw_vel, double pitch_vel, double yaw_acc, double pitch_acc,
                      uint8_t armor_name_num, double vx, double vy, double distance) {
    if (!cmd_publisher_) return;

    try {
        rm_interfaces::msg::GimbalCmd msg;
        const auto now_ns = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
        msg.header.frame_id = "";
        msg.fire_advice = control && fire;
        msg.big_yaw = control ? big_yaw : 0.0;
        msg.small_yaw = control ? small_yaw : 0.0;
        msg.pitch = control ? -pitch : 0.0;
        msg.yaw_vel_deg = control ? yaw_vel : 0.0;
        msg.pitch_vel_deg = control ? -pitch_vel : 0.0;
        msg.yaw_acc_deg = control ? yaw_acc : 0.0;
        msg.pitch_acc_deg = control ? -pitch_acc : 0.0;
        msg.enemy_id = armor_name_num;
        msg.vx = vx;
        msg.vy = vy;
        msg.target_distance = distance;
        cmd_publisher_->publish(msg);
    } catch (const std::exception& e) {
        tools::logger()->warn("[ROS2Gimbal] Failed to publish gimbal command: {}", e.what());
    }
}
}  // namespace io
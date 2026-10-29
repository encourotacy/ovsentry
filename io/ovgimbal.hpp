#ifndef IO__OVGIMBAL_HPP
#define IO__OVGIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <list>
#include <mutex>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io {
struct OVGimbalState {
    double yaw;           ///< 云台 yaw，单位为 rad
    double yaw_vel;       ///< 云台 yaw 角速度，单位为 rad/s
    double pitch;         ///< 云台 pitch 角，单位为 rad
    double pitch_vel;     ///< 云台 pitch 角速度，单位为 rad/s
    double bullet_speed;  ///< 弹速，单位为 m/s
};

enum class AutoAimMode : uint8_t {
    normal = 0x00,        ///< 普通模式，对应模式字节 0x00。通常不会发送。
    small_energy = 0x01,  ///< 小符模式，对应模式字节 0x01。
    big_energy = 0x02,    ///< 大符模式，对应模式字节 0x02。
    anti_top = 0x03,      ///< 自瞄/反陀螺模式，对应模式字节 0x03。
    unknown = 0xff        ///< 未知模式，对应无法识别的模式字节。
};

/* 云台通信类，负责与C板通过串口通信 */
class OVGimbal {
   public:
    explicit OVGimbal(const std::string& config_path);
    ~OVGimbal();

    // 向C板发送数据
    void send(const io::Command& c);
    void send(const io::MPCCommand& c);
    void send(bool control, bool fire, float yaw, float pitch, float yaw_vel, float yaw_acc,
              float pitch_vel, float pitch_acc, uint8_t armor_name_num = 0);  ///< 手动组织发送数据

    // 数据访问
    Eigen::Quaterniond q_at(
        std::chrono::steady_clock::time_point image_timestamp);  ///< 返回这个时间的IMU四元数
    AutoAimMode mode() const;       ///< 返回 AutoAimMode 类型的自瞄状态
    std::string mode_name() const;  ///< 返回字符串类型的自瞄状态
    OVGimbalState state() const;    ///< 返回云台状态
    bool has_imu() const;           ///< 返回是否已正常接收到 IMU/电控数据
    double offset_ms() const;       ///< 返回串口姿态固定时间补偿（单位：ms）
    double bullet_speed() const;    ///< 返回弹速
    uint8_t mode_byte() const;      ///< 返回表示 AutoAimMode 的十六进制数
    uint8_t invincibility() const;  ///< 返回敌方无敌状态串

   private:
    struct IMUData {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
    };
    tools::ThreadSafeQueue<IMUData> queue_{1000};

    serial::Serial serial_;          // 串口
    std::thread read_thread_;        // 读取电控数据的线程
    std::atomic<bool> quit_{false};  // 退出标志
    mutable std::mutex mutex_;       // 互斥锁

    std::chrono::microseconds imu_query_offset_{-1000};  // 一个延迟，用于补偿通信间的时间差

    // 三个数据组成"滑动窗口"，用于计算给定时刻四元数
    // ——————————————————————————————————————————————————————————————> t
    // data_prev_   data_ahead_   data_behind_
    IMUData data_ahead_;
    IMUData data_behind_;
    IMUData data_prev_;
    IMUData latest_data_;
    bool has_prev_{false};

    // 读取来自电控的数据
    std::atomic<bool> is_receiving_data_{false};
    uint8_t mode_byte_{0};
    double yaw_ = 0.0;
    double pitch_ = 0.0;
    double yaw_vel_ = 0.0;
    double pitch_vel_ = 0.0;
    double bullet_speed_;
    uint8_t invincibility_ = 0;  // 读取一个8位二进制，每一位表示一种机器人是否无敌

    void receiving_date();                          // 接收线程的函数
    bool read_exact(uint8_t* buffer, size_t size);  // 从串口读取一定长度的数据，并返回是否成功
    void reconnect();  // 当多次读取错误时，尝试重新连接串口
};

}  // namespace io

#endif  // IO__OVGIMBAL_HPP
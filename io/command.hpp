#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io {
/**
 * @brief 普通自瞄指令
 * @details 只发送瞄准需要到达的云台位置等基本信息
 */
struct Command {
    bool control;            ///< 是否进行控制，即是否瞄准到目标
    bool shoot;              ///< 是否开火
    double yaw;              ///< 指令 yaw 角，单位为 rad
    double pitch;            ///< 指令 pitch 角，单位为 rad
    uint8_t armor_name_num;  ///< 当前瞄准单位序号

    double big_yaw;    ///< 大 yaw，部分车型使用
    double small_yaw;  ///< 小 yaw，部分车型使用

    void rad2degree() {  ///< 弧度转角度
        yaw *= 57.3;
        pitch *= 57.3;
        big_yaw *= 57.3;
        small_yaw *= 57.3;
    }
};

/**
 * @brief MPC自瞄指令
 * @details 在普通指令基础上，发送速度和加速度供电控使用计算
 */
struct MPCCommand {
    bool control;  ///< 是否进行控制，即是否瞄准到目标
    bool shoot;    ///< 是否开火

    double yaw;              ///< 指令 yaw 角，单位为 rad
    double yaw_vel;          ///< 指令 pitch 角速度，单位为 rad/s
    double yaw_acc;          ///< 指令 pitch 角加速度，单位为 rad/s^2
    double pitch;            ///< 指令 pitch 角，单位为 rad
    double pitch_vel;        ///< 指令 pitch 角速度，单位为 rad/s
    double pitch_acc;        ///< 指令 pitch 角加速度，单位为 rad/s^2
    uint8_t armor_name_num;  ///< 当前瞄准单位序号

    double big_yaw;    ///< 大 yaw，部分车型使用
    double small_yaw;  ///< 小 yaw，部分车型使用
    double vx = 0;     ///< 目标 x 方向速度，单位为 m/s
    double vy = 0;     ///< 目标 y 方向速度，单位为 m/s
    double distance = 0;  ///< 目标水平距离，单位为 m

    void rad2degree() {  ///< 弧度转角度
        yaw *= 57.3;
        yaw_vel *= 57.3;
        yaw_acc *= 57.3;
        pitch *= 57.3;
        pitch_vel *= 57.3;
        pitch_acc *= 57.3;
        big_yaw *= 57.3;
        small_yaw *= 57.3;
    }
};

}  // namespace io

#endif  // IO__COMMAND_HPP
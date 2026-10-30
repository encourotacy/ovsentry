#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/ros2/ros2_gimbal.hpp"
#include "io/ros2/subscribe.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/ovsentry_omni_logic.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"

const std::string cli_keys =
    "{help h usage ? |                           | 输出命令行参数说明}"
    "{@config-path   | configs/sentry.yaml       | 位置参数，yaml配置文件路径 }";

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, cli_keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    /* === 创建各种类 === */
    tools::logger()->info("[OVSentry Standard MPC] auto aim = mpc, buff aim = arm-only");
    tools::Exiter exiter;  // 退出检测器，检测到Ctrl+C则会修改一个标记以表示需要退出
    tools::Recorder recorder;  // 内录，记录画面与四元数以便之后回放

    io::ROS2Gimbal gimbal(config_path);  // 云台类，接收和发送数据
    io::ArmorIgnoreSubscriber armor_ignore(config_path);
    io::StateSubscriber gimbal_mode(config_path);
    io::Camera camera(config_path);      // 相机，读取相机画面

    auto_aim::YOLO yolo(config_path);
    auto_aim::Solver aim_solver(config_path);
    auto_aim::Tracker tracker(config_path, aim_solver);
    auto_aim::Planner planner(config_path);

    omniperception::OmniSearch omni(config_path);
    bool prev_lost = false;

    /* === 主程序 === */
    tools::logger()->info("[OVSentry Standard MPC] started");

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    io::MPCCommand command{};
    int frame_count = 0;
    auto last_mode = io::ROS2Mode::auto_aim;

    while (!exiter.exit()) {
        //--- 读取相机画面 ---
        try {
            camera.read(img, timestamp);
            if (img.empty()) {
                tools::logger()->warn("[OVSentry Standard MPC] 读取到空图像，跳过此帧");
                continue;
            }
        } catch (const std::exception& e) {
            tools::logger()->error("[OVSentry Standard MPC] 读取摄像机图像失败: {}", e.what());
            continue;
        }
        frame_count++;
        omni.read_cameras();

        //--- 读取 IMU 四元数 ---
        const auto q = gimbal.q_at(timestamp);
        aim_solver.set_R_gimbal2world(q);
        // buff_solver.set_R_gimbal2world(q);

        // recorder.record(img, q, timestamp);  // 如果需要取消内录，将这行注释掉即可

        //--- 解析模式 ---
        const auto mode = gimbal_mode.mode();
        if (mode != last_mode) {
            tools::logger()->info("[OVSentry Standard MPC] switch mode={}", gimbal_mode.mode_name());
            last_mode = mode;
        }

        if (mode == io::ROS2Mode::auto_aim) {
            //--- 自瞄模式，计算自瞄指令 ---
            auto armors = yolo.detect(img, frame_count);
            tracker.filter(armors, cv::Point2f(img.rows / 2, img.cols / 2), 0);

            auto targets = tracker.track(armors, timestamp);

            // 全向感知：主相机 lost 时用侧相机搜索，状态切换时重置转向会话
            const auto tracker_state = tracker.state();
            const auto gimbal_state = gimbal.state();
            const bool lost = tracker_state == "lost";
            if (lost != prev_lost) omni.reset();
            prev_lost = lost;

            if (lost) {
                const auto ignore_mask = armor_ignore.ignore_ids();
                const auto steer = omni.update(
                    gimbal_state.big_yaw,
                    [&](std::chrono::steady_clock::time_point t) {
                        return gimbal.big_yaw_at_image(t);
                    },
                    [&](uint8_t id) {
                        return id >= 1 && id <= 8 &&
                               ((ignore_mask >> (id - 1)) & 0x1) != 0;
                    },
                    frame_count);
                command = {};
                command.control = steer.control;
                command.small_yaw = steer.small_yaw;
                command.big_yaw = steer.big_yaw;
                command.pitch = steer.pitch;
                command.armor_name_num = steer.armor_id;
                command.rad2degree();
                gimbal.send(command);
            } else if (tracker_state == "detecting") {
                command = {};
                command.control = true;
                command.shoot = false;
                command.small_yaw = gimbal_state.small_yaw;
                command.big_yaw = gimbal_state.big_yaw;
                command.pitch = -gimbal_state.pitch;
                command.rad2degree();
                gimbal.send(command);
            } else {
                std::optional<auto_aim::Target> target_for_plan =
                    targets.empty() ? std::nullopt : std::make_optional(targets.front());
                command = planner.plan(target_for_plan, gimbal.bullet_speed());
                command.small_yaw = command.yaw;
                command.big_yaw = command.yaw;
                if (command.control && !targets.empty()) {
                    const auto& target = targets.front();
                    const auto ekf_x = target.ekf_x();
                    command.vx = ekf_x[1];
                    command.vy = ekf_x[3];
                    command.distance = std::hypot(ekf_x[0], ekf_x[2]);
                    if (target.name == auto_aim::ArmorName::OUTPOST) {
                        command.big_yaw = gimbal_state.big_yaw;
                    } else {
                        const double target_yaw = std::atan2(ekf_x[2], ekf_x[0]);
                        const double yaw_error = tools::limit_rad(target_yaw - gimbal_state.big_yaw);
                        command.big_yaw = gimbal_state.big_yaw + yaw_error;
                    }
                }
                command.rad2degree();

                gimbal.send(command);
            }

        } else if (mode == io::ROS2Mode::small_buff || mode == io::ROS2Mode::big_buff) {
            if (prev_lost) omni.reset();
            prev_lost = false;
            gimbal.send(false, false, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
        } else {
            if (prev_lost) omni.reset();
            prev_lost = false;
            gimbal.send(false, false, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
        }
    }

    gimbal.send(false, false, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);

    return 0;
}

#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
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
    tools::logger()->info("[OVSentry MPC Debug] auto aim = mpc, buff aim = arm-only");
    tools::Plotter plotter;  // 曲线绘图类，发送 data 以供显示调试数据曲线
    tools::Exiter exiter;  // 退出检测器，检测到Ctrl+C则会修改一个标记以表示需要退出
    tools::Recorder recorder;  // 内录，记录画面与四元数以便之后回放
    nlohmann::json data;       // 数据记录用

    io::ROS2Gimbal gimbal(config_path);  // 云台类，接收和发送数据
    io::ArmorIgnoreSubscriber armor_ignore(config_path);
    io::StateSubscriber gimbal_mode(config_path);
    io::Camera camera(config_path);      // 相机，读取相机画面

    auto_aim::YOLO yolo(config_path, true);
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
    std::string chain_name = "";  // 调试画面显示用
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
                chain_name = "omni";
            } else if (tracker_state == "detecting") {
                command = {};
                command.control = true;
                command.shoot = false;
                command.small_yaw = gimbal_state.small_yaw;
                command.big_yaw = gimbal_state.big_yaw;
                command.pitch = -gimbal_state.pitch;
                command.rad2degree();
                gimbal.send(command);
                chain_name = "detecting";
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

                //--- 调试画图 ---
                chain_name = "auto_aim";
                if (!armors.empty()) {
                    for (auto& armor : armors) {  // 绘制 solver 重投影后的所有目标装甲板的位置。
                        aim_solver.solve(armor);
                        auto image_points = aim_solver.reproject_armor(
                            armor.xyz_in_world_, armor.ypr_in_world_[0], armor.name_);
                        tools::draw_points(img, image_points, {255, 0, 0});  // 显示为蓝色
                    }

                    // data["armor_num"] = armors.size();
                    // const auto & armor = armors.front();
                    // data["armor_x"] = armor.xyz_in_world_[0];
                    // data["armor_y"] = armor.xyz_in_world_[1];
                    // data["armor_yaw"] = armor.ypr_in_world_[0] * 57.3;
                    // data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
                    // data["armor_center_x"] = armor.center_norm_.x;
                }

                if (!targets.empty()) {
                    auto target = targets.front();
                    auto armor_xyza_list = target.armor_xyza_list();

                    for (const Eigen::Vector4d& xyza :
                         armor_xyza_list) {  // 绘制预测的 target 的各个装甲板
                        auto image_points =
                            aim_solver.reproject_armor(xyza.head(3), xyza[3], target.name);
                        tools::draw_points(img, image_points, {0, 255, 0});  // 显示为绿色
                    }

                    if (command.control) {  // 绘制正在瞄准的位置
                        Eigen::Vector4d aim_xyza = planner.aim_xyza;
                        auto image_points =
                            aim_solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.name);
                        tools::draw_points(img, image_points, {0, 0, 255});  // 显示为红色
                    }

                    // Eigen::VectorXd x = target.ekf_x();
                    // data["x"] = x[0];
                    // data["vx"] = x[1];
                    // data["y"] = x[2];
                    // data["vy"] = x[3];
                    // data["z"] = x[4];
                    // data["vz"] = x[5];
                    // data["a"] = x[6] * 57.3;
                    // data["w"] = x[7];
                    // data["r"] = x[8];
                    // data["l"] = x[9];
                    // data["h"] = x[10];

                    // 卡方检验数据
                    // data["innovation_yaw"] = target.ekf().data.at("innovation_yaw");
                    // data["innovation_pitch"] = target.ekf().data.at("innovation_pitch");
                    // data["innovation_distance"] = target.ekf().data.at("innovation_distance");
                    // data["innovation_angle"] = target.ekf().data.at("innovation_angle");
                    // data["nis"] = target.ekf().data.at("nis");
                    // data["nis_fail"] = target.ekf().data.at("nis_fail");
                    // data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
                }

                if (command.control) {
                    data["shoot"] = command.shoot;
                    data["cmd_yaw"] = command.small_yaw;
                    data["cmd_yaw_vel"] = command.yaw_vel;
                    data["cmd_yaw_acc"] = command.yaw_acc;
                    data["cmd_pitch"] = command.pitch;
                    data["cmd_pitch_vel"] = command.pitch_vel;
                    data["cmd_pitch_acc"] = command.pitch_acc;
                    data["armor_name_num"] = command.armor_name_num;
                }
            }

        } else if (mode == io::ROS2Mode::small_buff || mode == io::ROS2Mode::big_buff) {
            if (prev_lost) omni.reset();
            prev_lost = false;
            gimbal.send(false, false, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0);
            //--- 调试画图 ---
            chain_name = "auto_buff";
        } else {
            if (prev_lost) omni.reset();
            prev_lost = false;
            chain_name = "unknow";
            gimbal.send(false, false, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0);
        }

        //--- 显示与发送调试数据 ---
        const auto gs = gimbal.state();
        data["gimbal_small_yaw"] = tools::rad2deg(gs.small_yaw);
        data["gimbal_pitch"] = tools::rad2deg(gs.pitch);
        data["gimbal_yaw_vel"] = tools::rad2deg(gs.yaw_vel);
        data["gimbal_pitch_vel"] = tools::rad2deg(gs.pitch_vel);
        data["gimbal_big_yaw"] = tools::rad2deg(gs.big_yaw);

        auto mode_str = fmt::format("Mode: {} | Chain: {}", gimbal_mode.mode_name(), chain_name);
        auto gs_str = fmt::format("Gimbal: small_yaw={:.1f}, pitch={:.1f}, bullet={:.1f}", 
                                tools::rad2deg(gs.small_yaw), tools::rad2deg(gs.pitch), gs.bullet_speed);
        auto cmd_str = fmt::format("Cmd: control={}, small_yaw={:.2f}, pitch={:.2f}, shoot={}",
                                   command.control, command.small_yaw, command.pitch, command.shoot);

        tools::draw_text(img, mode_str, {10, 30}, {255, 255, 255});
        tools::draw_text(img, gs_str, {10, 60}, {255, 255, 255});
        tools::draw_text(img, cmd_str, {10, 90},
                         command.control ? cv::Scalar(154, 50, 205) : cv::Scalar(120, 120, 120));
        tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 120}, {0, 255, 0});

        plotter.plot(data);
        cv::imshow("OVSentry Standard MPC Debug", img);
        for (const auto& frame : omni.frames()) {
            if (frame.image.empty()) continue;
            cv::Mat shown;
            cv::resize(frame.image, shown, cv::Size(), 0.5, 0.5);
            cv::imshow(frame.name, shown);
        }
        auto key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') break;
    }

    gimbal.send(false, false, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0);

    return 0;
}

#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/ovgimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string cli_keys =
    "{help h usage ? |                           | 输出命令行参数说明}"
    "{@config-path   | configs/standard3.yaml   | 位置参数，yaml配置文件路径 }";

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, cli_keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    /* === 创建各种类 === */
    tools::logger()->info("[OVGimbal Standard MPC] auto aim=mpc, buff aim=arm-only");
    tools::Exiter exiter;  // 退出检测器，检测到Ctrl+C则会修改一个标记以表示需要退出
    tools::Recorder recorder;  // 内录，记录画面与四元数以便之后回放

    io::OVGimbal gimbal(config_path);  // 云台类，接收和发送数据
    io::Camera camera(config_path);    // 相机，读取相机画面

    auto_aim::YOLO yolo(config_path);
    auto_aim::Solver aim_solver(config_path);
    auto_aim::Tracker tracker(config_path, aim_solver);
    auto_aim::Planner planner(config_path);

    /* === 主程序 === */
    tools::logger()->info("[OVGimbal Standard MPC] started");

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    io::MPCCommand command = {false, false, 0, 0, 0, 0, 0, 0, 0};
    int frame_count = 0;
    auto last_mode = io::AutoAimMode::unknown;

    while (!exiter.exit()) {
        //--- 读取相机画面 ---
        try {
            camera.read(img, timestamp);
            if (img.empty()) {
                tools::logger()->warn("[OVGimbal Standard MPC] 读取到空图像，跳过此帧");
                continue;
            }
        } catch (const std::exception& e) {
            tools::logger()->error("[OVGimbal Standard MPC] 读取摄像机图像失败: {}", e.what());
            continue;
        }
        frame_count++;

        //--- 读取 IMU 四元数 ---
        const auto q = gimbal.q_at(timestamp);
        aim_solver.set_R_gimbal2world(q);
        // buff_solver.set_R_gimbal2world(q);

        // recorder.record(img, q, timestamp);  // 如果需要取消内录，将这行注释掉即可

        //--- 解析模式 ---
        const auto mode = gimbal.mode();
        if (mode != last_mode) {
            tools::logger()->info("[OVGimbal Standard MPC] switch mode={} raw=0x{:02X}",
                                  gimbal.mode_name(), gimbal.mode_byte());
            last_mode = mode;
        }

        if (mode == io::AutoAimMode::normal || mode == io::AutoAimMode::anti_top) {
            //--- 自瞄模式，计算自瞄指令 ---
            auto armors = yolo.detect(img, frame_count);
            tracker.filter(armors, cv::Point2f(img.rows / 2, img.cols / 2), 0);

            auto targets = tracker.track(armors, timestamp);

            std::optional<auto_aim::Target> target_for_plan =
                targets.empty() ? std::nullopt : std::make_optional(targets.front());
            command = planner.plan(target_for_plan, gimbal.bullet_speed());
            command.rad2degree();

            gimbal.send(command);
        } else if (mode == io::AutoAimMode::small_energy || mode == io::AutoAimMode::big_energy) {
            gimbal.send(false, false, 0, 0, 0, 0, 0, 0, 0);
        } else {
            gimbal.send(false, false, 0, 0, 0, 0, 0, 0, 0);
        }
    }

    gimbal.send(false, false, 0, 0, 0, 0, 0, 0, 0);

    return 0;
}

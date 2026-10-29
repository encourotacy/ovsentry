#include "tasks/auto_aim/tracker.hpp"

#include <fmt/core.h>

#include <chrono>
#include <fstream>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

const std::string keys =
    "{help h usage ? |                   | 输出命令行参数说明 }"
    "{config-path c  | configs/standard3.yaml | yaml配置文件的路径}"
    "{start-index s  | 0                 | 视频起始帧下标    }"
    "{end-index e    | 0                 | 视频结束帧下标    }"
    "{@input-path    | recorder/  | avi和txt文件的路径}";

// ./build/tracker_test ./records/5-23_qdu --start-index=10000

int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    auto input_path = cli.get<std::string>(0);
    auto config_path = cli.get<std::string>("config-path");
    auto start_index = cli.get<int>("start-index");
    auto end_index = cli.get<int>("end-index");

    // 创建类
    tools::Plotter plotter;
    tools::Exiter exiter;
    auto_aim::YOLO yolo(config_path);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    nlohmann::json data;  // 数据记录用

    // 打开视频和txt文件
    auto video_path = fmt::format("{}.avi", input_path);
    auto text_path = fmt::format("{}.txt", input_path);
    cv::VideoCapture video(video_path);
    std::ifstream text(text_path);

    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
    for (int i = 0; i < start_index; i++) {
        double t, w, x, y, z;
        text >> t >> w >> x >> y >> z;
    }

    // 主程序
    cv::Mat img;
    auto t0 = std::chrono::steady_clock::now();
    for (int frame_count = start_index; !exiter.exit(); frame_count++) {
        if (end_index > 0 && frame_count > end_index) break;

        // 读取数据
        video.read(img);
        if (img.empty()) break;

        double t, w, x, y, z;
        text >> t >> w >> x >> y >> z;
        auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

        // 测试主要内容
        solver.set_R_gimbal2world({w, x, y, z});

        auto armors = yolo.detect(img, frame_count);
        tracker.filter(armors, cv::Point2f(img.rows / 2, img.cols / 2), 0);

        auto targets = tracker.track(armors, timestamp);

        tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 120}, {0, 255, 0});

        uint8_t armor_id = 0;
        if (!targets.empty()) {
            armor_id = static_cast<uint8_t>(targets.front().name);
            if (armor_id == 0) armor_id = static_cast<uint8_t>(5);
            if (armor_id == 5) armor_id = static_cast<uint8_t>(0);
        }

        // 画图
        if (!armors.empty()) {
            for (auto& armor : armors) {
                solver.solve(armor);
                auto image_points = solver.reproject_armor(armor.xyz_in_world_,
                                                           armor.ypr_in_world_[0], armor.name_);
                tools::draw_points(img, image_points, {255, 0, 0});
            }
        }

        if (!targets.empty()) {
            auto target = targets.front();
            auto armor_xyza_list = target.armor_xyza_list();

            for (const Eigen::Vector4d& xyza : armor_xyza_list) {
                auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.name);
                tools::draw_points(img, image_points, {0, 255, 0});
            }

            // 卡方检验数据
            data["innovation_yaw"] = target.ekf().data.at("innovation_yaw");
            data["innovation_pitch"] = target.ekf().data.at("innovation_pitch");
            data["innovation_distance"] = target.ekf().data.at("innovation_distance");
            data["innovation_angle"] = target.ekf().data.at("innovation_angle");
            data["nis"] = target.ekf().data.at("nis");
            data["nis_fail"] = target.ekf().data.at("nis_fail");
            data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
        }

        tools::draw_text(img, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

        data["hello"] = 1;
        plotter.plot(data);
        cv::imshow("result", img);
        // 退出与暂停
        auto key = cv::waitKey(33);
        if (key == 'q' || key == 'Q') break;
        while (key == ' ') {
            int pause_key = cv::waitKey(30);
            if (pause_key == 'q' || pause_key == 'Q') {
                key = pause_key;
                break;
            }
            if (pause_key == ' ') break;
        }
        if (key == 'q' || key == 'Q') break;
    }

    return 0;
}
#include "tasks/auto_aim/yolo.hpp"

#include <fmt/core.h>

#include <chrono>
#include <fstream>

#include "tools/exiter.hpp"

const std::string keys =
    "{help h usage ? |                   | 输出命令行参数说明 }"
    "{config-path c  | configs/standard3.yaml | yaml配置文件的路径}"
    "{start-index s  | 0                 | 视频起始帧下标    }"
    "{end-index e    | 0                 | 视频结束帧下标    }"
    "{@input-path    | recorder/  | avi和txt文件的路径}";

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
    tools::Exiter exiter;
    auto_aim::YOLO yolo(config_path, true);

    auto video_path = fmt::format("{}.avi", input_path);
    cv::VideoCapture video(video_path);
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);

    // 主程序
    cv::Mat img;
    for (int frame_count = start_index; !exiter.exit(); frame_count++) {
        if (end_index > 0 && frame_count > end_index) break;

        video.read(img);
        if (img.empty()) break;

        auto armor = yolo.detect(img, frame_count);

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
#include "buff_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <string>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace auto_buff {
//=== 初始化 ===
Buff_Detector::Buff_Detector(const std::string& config_path, const std::string& model_key)
    : yolo_model_(config_path, model_key),
      // energy_type_(EnergyType::SMALL),
      // energy_color_(BuffColor::RED),
      status_(TrackStatus::LOSE),
      last_powerrune(std::nullopt),
      lose_(0) {
    // auto yaml = tools::Load(config_path);
    // //--- 读取并转换颜色 ---
    // std:::string enemy_color_name = tools::read<std::string>(yaml, "enemy_color");
    // std::transform(enemy_color_name.begin(), enemy_color_name.end(), enemy_color_name.begin(),
    //     [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    // energy_color_ = enemy_color_name == "blue" ? BuffColor::RED : BuffColor::BLUE ;
    // tools::logger()->info(
    //     "[Buff_Detector] enemy_color={}, buff_target_color={}",
    //     enemy_color_name, enemy_color_name == "blue" ? "red" : "blue");
}

//=== 主程序 ===
std::optional<PowerRune> Buff_Detector::handle_target_lose() {
    lose_++;
    if (lose_ >= LOSE_MAX) {
        status_ = TrackStatus::LOSE;
        last_powerrune_ = std::nullopt;
        prediction_valid_ = false;
        if (lose_ == LOSE_MAX)
            tools::logger()->debug("[Detector] 目标彻底丢失(丢失{}帧以上)，已重置", LOSE_MAX);
    } else if (lose_ < PREDICT_LOSE_THRESH && last_powerrune_.has_value() && target_ != nullptr &&
               !target_->is_unsolve()) {
        //...预测跟踪模式
        if (lose_ < 10 || lose_ > 18)
            tools::logger()->debug("[Detector] 预测跟踪模式，丢失{}帧, dt={:.3f}s", lose_, dt);
    } else {
        status_ = TrackStatus::TEM_LOSE;
        prediction_valid_ = false;
        if (lose == PREDICT_LOSE_THRESH) tools::logger()->debug("[Detector] 目标临时丢失！");
    }

    if (status_ == TrackStatus::PREDICT_TRACK && prediction_valid_) {
        return std::optional<PowerRune>(predicted_powerrune_);
    }
    return std::nullopt;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat& bgr_img) {
    timestamp_ = std::chrono::steady_clock::now();

    //--- 获取目标 ---
    std::vector<YOLO11_BUFF::Object> results = yolo_model_.get_candidateboxes(bgr_img);
    // 这里可以插入颜色区分，但说实话感觉没必要doge
    if (results.empty()) return handle_target_lose();

    //--- 创建新的能量机关类 ---
    std::vector<FanBlade> fanblades;
    for (const auto& result : results) fanblades.emplace_back(FanBlade(result.kpt));
    auto r_center = get_r_center(fanblades, bgr_img);
    PowerRune powerrune(fanblades, r_center, last_powerrune_);
    if (powerrune.is_unsolvable()) return handle_target_lose();

    //--- 返回 ---
    lose_ = 0;
    status_ = TrackState::TRACK;
    prediction_valid_ = false;
    std::optional<PowerRune> P;
    P.emplace(powerrune);
    last_powerrune_ = P;
    return P;
}
//=== 计算能量机关中心点 ===
cv::Point2f Buff_Detector::get_r_center(const std::vector<FanBlade>& fanblades,
                                        const cv::Mat& bgr_img) {
    if (fanblades.empty()) {  // 理论上不会触发这个，以防万一写一下
        tools::logger()->warn("[Buff_Detector] 计算 r_center 时输入空目标！");
        return {0, 0};
    }

    //--- 利用关键点估计 r_center ---
    cv::Point2f r_center_t = {0, 0};
    for (auto& fanblade : fanblades) {
        auto point5 = fanblade.center;
        auto point6 = fanblade.kpts[5];
        r_center_t += (point6 - point5) * 70 / 52 + point5;  // 5->6:520mm, 5->center:700mm
    }
    r_center_t /= float(fanblades.size());

    //--- 二值化图片，并只保留r_center_t周围的图像 ---
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
    cv::Mat dilated_img;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 一个"画笔"
    cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1),
               1);  // 膨胀，将白色部分向外"蔓延"

    double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
    // 扇叶上靶子半径为 150mm, 能量机关中心半径 53mm.
    // 乘0.8以有较大可能包含能量机关中心，同时范围也不大
    cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // 一张空图（即全黑）
    cv::circle(mask, r_center_t, radius, cv::Scalar(255),
               -1);  // 在 r_center_t 处画一个r=radius的白色实心圆
    cv::bitwise_and(dilated_img, mask,
                    dilated_img);  // dilated_img与mask每一位做与运算，即只保留圆中的图像

    //--- 找出最可能的中心 ---
    std::vector<std::vector<cv::Point>> contours;
    auto r_center = r_center_t;
    cv::findContours(dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    // cv::RETR_EXTERNAL表示只找最外层的轮廓

    double best_score = std::numeric_limits<double>::infinity();
    ;
    for (auto& contour : contours) {
        auto rotated_rect = cv::minAreaRect(contour);
        double ratio = rotated_rect.size.height > rotated_rect.size.width
                           ? rotated_rect.size.height / rotated_rect.size.width
                           : rotated_rect.size.width / rotated_rect.size.height;
        // 能量机关中心是一个圆形，所以一个框越接近正方形越可能是中心
        double score = ratio + cv::norm(rotated_rect.center - r_center_t) / (radius / 3);

        if (score < best_score) {
            best_score = score;
            r_center = rotated_rect.center;
        }
    }

    return r_center;
}

}  // namespace auto_buff

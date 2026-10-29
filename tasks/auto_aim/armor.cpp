#include "armor.hpp"

#include <cmath>

#include "tools/logger.hpp"

namespace auto_aim {

//=== 灯条构造函数 ===
Lightbar::Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id)
    : id(id), rotated_rect(rotated_rect) {
    // 这里默认灯条是竖着的，因此将 rotated_rect 中的角点重新排序了
    std::vector<cv::Point2f> corners(4);
    rotated_rect.points(&corners[0]);
    std::sort(corners.begin(), corners.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    // 开始计算各种位置点
    center = rotated_rect.center;
    top = (corners[0] + corners[1]) / 2;
    bottom = (corners[2] + corners[3]) / 2;
    top2bottom_vec = bottom - top;

    points.emplace_back(top);
    points.emplace_back(bottom);

    length = cv::norm(top2bottom_vec);
    width = cv::norm(corners[0] - corners[1]);
    ratio = length / width;
    angle = std::atan2(top2bottom_vec.y, top2bottom_vec.x);
    angle_error = std::abs(angle - CV_PI / 2);
}

//=== 装甲板构造函数 ===
Armor::Armor(const Lightbar& left, const Lightbar& right)
    : left_light_(left),
      right_light_(right),
      color_(left.color),
      center_((left.center + right.center) / 2),
      duplicated_(false) {  // 传统构造函数
    points_.emplace_back(left.top);
    points_.emplace_back(right.top);
    points_.emplace_back(right.bottom);
    points_.emplace_back(left.bottom);

    auto left2right_vec = right.center - left.center;
    auto width = cv::norm(left2right_vec);
    auto max_lightbar_length = std::max(left.length, right.length);
    auto min_lightbar_length = std::min(left.length, right.length);
    ratio_ = width / max_lightbar_length;
    side_ratio_ = max_lightbar_length / min_lightbar_length;

    auto roll = std::atan2(left2right_vec.y, left2right_vec.x);
    auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
    auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
    rectangular_error_ = std::max(left_rectangular_error, right_rectangular_error);
}

Armor::Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
             std::vector<cv::Point2f> armor_kpts, cv::Point2f offset)
    : confidence_(confidence), box_(box), points_(armor_kpts) {  // YOLOv5构造函数
    if (offset.x != -1) {  // 不是默认值，说明启用了 ROI
        std::transform(armor_kpts.begin(), armor_kpts.end(), armor_kpts.begin(),
                       [&offset](const cv::Point2f& point) { return point + offset; });
        std::transform(points_.begin(), points_.end(), points_.begin(),
                       [&offset](const cv::Point2f& point) { return point + offset; });
    }

    center_ = (armor_kpts[0] + armor_kpts[1] + armor_kpts[2] + armor_kpts[3]) / 4;
    auto left_width = cv::norm(armor_kpts[0] - armor_kpts[3]);
    auto right_width = cv::norm(armor_kpts[1] - armor_kpts[2]);
    auto max_width = std::max(left_width, right_width);
    auto top_length = cv::norm(armor_kpts[0] - armor_kpts[1]);
    auto bottom_length = cv::norm(armor_kpts[3] - armor_kpts[2]);
    auto max_length = std::max(top_length, bottom_length);
    auto left_center = (armor_kpts[0] + armor_kpts[3]) / 2;
    auto right_center = (armor_kpts[1] + armor_kpts[2]) / 2;
    auto left2right = right_center - left_center;
    auto roll = std::atan2(left2right.y, left2right.x);
    auto left_rectangular_error =
        std::abs(std::atan2((armor_kpts[3] - armor_kpts[0]).y, (armor_kpts[3] - armor_kpts[0]).x) -
                 roll - CV_PI / 2);
    auto right_rectangular_error =
        std::abs(std::atan2((armor_kpts[2] - armor_kpts[1]).y, (armor_kpts[2] - armor_kpts[1]).x) -
                 roll - CV_PI / 2);
    rectangular_error_ = std::max(left_rectangular_error, right_rectangular_error);
    ratio_ = max_length / max_width;
    color_ = color_id == 0 ? Color::BLUE : color_id == 1 ? Color::RED : Color::DARK;
    name_ = ArmorName(num_id);

    type_ = (name_ == ArmorName::OUTPOST || name_ == ArmorName::BASE) ? ArmorType::BASE_OUTPOST
                                                                      : ArmorType::SMALL;
    try {
        priority_ = getPriority.at(name_);
    } catch (const std::exception& e) {
        tools::logger()->error("[Armor] Failed to get priority , name={}", name_);
        throw;
    }
}
}  // namespace auto_aim
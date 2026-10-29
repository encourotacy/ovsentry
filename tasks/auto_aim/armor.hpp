#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim {
/* 各种枚举类
 * 由于装甲板类的各种属性需要在整数和 enum 之间来回转换，所以这里不使用 enum class*/
enum Color { RED, BLUE, DARK, PURPLE };  // DARK表示熄灭
enum ArmorType { BIG, SMALL, BASE_OUTPOST };
enum ArmorName { SENTRY, ONE, TWO, THREE, FOUR, FIVE, OUTPOST, BASE, NOT_ARMOR };
const std::vector<std::string> COLORS = {"red", "blue", "dark", "purple"};
const std::vector<std::string> ARMOR_TYPES = {"big", "small", "base"};
const std::vector<std::string> ARMOR_NAMES = {"sentry", "one",     "two",  "three",    "four",
                                              "five",   "outpost", "base", "not_armor"};

enum ArmorPriority { FIRST = 1, SECOND, THIRD, FORTH, FIFTH };

const std::unordered_map<ArmorName, ArmorPriority> getPriority = {
    {ArmorName::SENTRY, ArmorPriority::FORTH},   {ArmorName::ONE, ArmorPriority::FIRST},
    {ArmorName::TWO, ArmorPriority::FORTH},      {ArmorName::THREE, ArmorPriority::SECOND},
    {ArmorName::FOUR, ArmorPriority::SECOND},    {ArmorName::FIVE, ArmorPriority::FIFTH},
    {ArmorName::OUTPOST, ArmorPriority::FIFTH},  {ArmorName::BASE, ArmorPriority::FIFTH},
    {ArmorName::NOT_ARMOR, ArmorPriority::FIFTH}};

/* 灯条类。
 * 这里有很多内容似乎没有用上，但是为了以防之后要用所以先算出来了*/
struct Lightbar {
    size_t id;
    Color color;
    cv::Point2f center, top, bottom, top2bottom_vec;
    std::vector<cv::Point2f> points;  // 存储了灯条上下两点
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;

    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id);
    Lightbar(){};
};

/* 装甲板类。
 * 由于装甲板类里的很多内容需要在之后的程序中计算和修改，这里用 struct 写。
 * 应该和 class 中全是 public 是等效的 */
struct Armor {
    // 属性
    Color color_;
    ArmorType type_;
    ArmorName name_;
    ArmorPriority priority_ = ArmorPriority::FIFTH;

    // 定位用
    Lightbar left_light_, right_light_;
    cv::Point2f center_;  // 不是对角线交点，不能作为实际中心！
    cv::Point2f center_norm_;
    std::vector<cv::Point2f> points_;  // 按左上、右上、右下、左下顺序存储的灯条的位置

    cv::Rect box_;
    cv::Mat pattern_;

    // 坐标变换相关
    Eigen::Vector3d xyz_in_gimbal_;  // 单位：m
    Eigen::Vector3d
        xyz_in_world_;  // 单位：m
                        // 需要注意的是，这是以云台坐标系为原点、方向和世界坐标系一致的坐标系下的坐标。
                        // 这个坐标系的原点是会随着机器人移动而移动的，不是世界坐标系哦。
                        // 具体为什么需要去看solver中这个变量的计算式。
    Eigen::Vector3d ypr_in_gimbal_;  // 单位：rad
    Eigen::Vector3d ypr_in_world_;   // 单位：rad
    Eigen::Vector3d ypd_in_world_;   // 球坐标系

    // 其他
    double ratio_;              // 两灯条的中点连线与长灯条的长度之比
    double side_ratio_;         // 长灯条与短灯条的长度之比
    double rectangular_error_;  // 灯条和中点连线所成夹角与π/2的差值
    double confidence_;         // 识别时的置信度
    bool duplicated_;
    double yaw_raw;  // PnP直接解算出yaw值

    // 构造方法
    Armor(const Lightbar& left, const Lightbar& right);  // 传统构造方法，使用灯条定位装甲板
    Armor(int color_id, int num_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_kpts,
          cv::Point2f offset = {-1.0, -1.0});  // YOLOv5构造方法
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
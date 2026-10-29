#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace {
const double CONFIDENCE_THRESH = 0.70;  // 置信度阈值，低于这个置信度则忽略目标
const double IOU_THRESH = 0.30;  // 交并比(IoU)阈值，高于这个阈值的两个框视为高度重叠
}  // namespace

namespace auto_aim {
//=== 初始化 ===
YOLOV5::YOLOV5(const std::string& config_path, bool debug) : debug_(debug) {
    auto yaml = YAML::LoadFile(config_path);
    device_ = yaml["device"].as<std::string>();
    model_path_ = yaml["yolov5_model_path"].as<std::string>();
    use_roi_ = yaml["use_roi"].as<bool>();
    use_traditional_ = yaml["use_traditional"].as<bool>();

    //--- 读取 ROI 参数 ---
    int x = 0, y = 0, width = 0, height = 0;
    x = yaml["roi"]["x"].as<int>();
    y = yaml["roi"]["y"].as<int>();
    width = yaml["roi"]["width"].as<int>();
    height = yaml["roi"]["height"].as<int>();
    roi_ = cv::Rect(x, y, width, height);
    offset_ = cv::Point2f(x, y);  // 由于识别的是截取的图片，需要这个来还原真实像素坐标

    // 创建文件夹，以保存用于训练的图片
    save_path_ = "imgs";
    std::filesystem::create_directory(save_path_);

    //--- 处理模型 ---
    auto model = core_.read_model(model_path_);   // 将模型加载进内存
    ov::preprocess::PrePostProcessor ppp(model);  // 用于定义推理前后要做什么处理
    auto& input = ppp.input();

    input.tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, 640, 640, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    // 描述输入张量的格式、大小等，这里的意思是：
    // 数据类型：unsigned 8(无符号8位，0~255)，
    // 形状：batch=1，宽640，高640，通道数3
    // 格式/布局：{batch,Height,Width,Channel}
    // 颜色顺序：BGR

    input.model().set_layout("NCHW");  // 设置内部期望的布局

    input.preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale(255.0);
    // 预处理步骤，这里的意思是：
    // 把 u8 转为 float 32
    // 把输入转为 RGB
    // 把像素值除以 255.0，也就是把0~255归一化为0~1

    model = ppp.build();  // 将上述步骤融合进模型里，返回一个新的 ov::Model
    compiled_model_ = core_.compile_model(
        model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}
//=== 主程序 ===
std::list<Armor> YOLOV5::detect(const cv::Mat& raw_img, int frame_count) {
    if (raw_img.empty()) {
        tools::logger()->warn("[YOLOV5] Received empty image!");
        return std::list<Armor>();
    }

    //--- 根据 ROI 裁切图片 ---
    cv::Mat processed_img;
    if (use_roi_) {
        if (roi_.width == -1) {  // -1 表示该维度不裁切
            roi_.width = raw_img.cols;
        }
        if (roi_.height == -1) {  // -1 表示该维度不裁切
            roi_.height = raw_img.rows;
        }
        processed_img = raw_img(roi_);
    } else {
        processed_img = raw_img;
    }

    //--- 压缩图片 ---
    auto x_scale = static_cast<double>(640) / processed_img.rows;
    auto y_scale = static_cast<double>(640) / processed_img.cols;
    auto scale = std::min(x_scale, y_scale);
    // 计算缩放倍数，取较小值以使得画面缩放后恰好能存在640*640区域内
    auto h = static_cast<int>(processed_img.rows * scale);
    auto w = static_cast<int>(processed_img.cols * scale);
    // 缩放后图片的尺寸
    auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(processed_img, input(roi), {w, h});

    //--- 输入图片并推理 ---
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    //--- 获取输出层 ---
    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5::parse(double scale, cv::Mat& output, const cv::Mat& raw_img,
                               int frame_count) {
    //--- 从 output 中读取数据 ---
    std::vector<int> color_ids, num_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;
    for (int r = 0; r < output.rows; r++) {
        double score = output.at<float>(r, 8);
        score = sigmoid(score);
        if (score < CONFIDENCE_THRESH) continue;

        std::vector<cv::Point2f> armor_key_points;
        cv::Mat color_scores = output.row(r).colRange(9, 13);     // color
        cv::Mat classes_scores = output.row(r).colRange(13, 22);  // num
        cv::Point class_id, color_id;
        int _class_id, _color_id;
        cv::minMaxLoc(classes_scores, NULL, NULL, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, NULL, NULL, &color_id);
        _class_id = class_id.x;
        _color_id = color_id.x;

        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

        float min_x = armor_key_points[0].x;
        float max_x = armor_key_points[0].x;
        float min_y = armor_key_points[0].y;
        float max_y = armor_key_points[0].y;

        for (int i = 1; i < armor_key_points.size(); i++) {
            if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
            if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
            if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
            if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
        }

        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

        color_ids.emplace_back(_color_id);
        num_ids.emplace_back(_class_id);
        boxes.emplace_back(rect);
        confidences.emplace_back(score);
        armors_key_points.emplace_back(armor_key_points);
    }

    //--- 去重，组合数据生成Armor类 ---
    std::vector<int> indexes;  // 存储保留的候选框的索引
    cv::dnn::NMSBoxes(boxes, confidences, CONFIDENCE_THRESH, IOU_THRESH, indexes);
    // 筛选出置信度大于CONFIDENCE_THRESH的框，并根据交并比去除高度重合的框。

    std::list<Armor> armors;
    for (const auto& i : indexes) {
        if (use_roi_) {
            armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i],
                                armors_key_points[i], offset_);
        } else {
            armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i],
                                armors_key_points[i]);
        }
    }

    //--- 检查识别结果与计算center_norm ---
    for (auto it = armors.begin(); it != armors.end();) {
        auto flag = it->name_ != ArmorName::NOT_ARMOR && it->confidence_ > CONFIDENCE_THRESH;
        if (!flag) {
            if (it->name_ != ArmorName::NOT_ARMOR) {
                save(raw_img, *it);  // 保存识别错误的图片用于迭代模型
            }
            it = armors.erase(it);
            continue;
        }
        // if (use_traditional_) detector_.detect(*it, raw_img); // 使用传统方法二次矫正角点
        it->center_norm_ = cv::Point2f(it->center_.x / raw_img.cols, it->center_.y / raw_img.rows);

        ++it;  // 放在这里是为了防止 erase 后又 ++ 而跳过了一个元素
    }

    if (debug_) draw_detections(raw_img, armors, frame_count);

    return armors;
}

//=== 工具方法 ===
double YOLOV5::sigmoid(double x) {
    if (x > 0)
        return 1.0 / (1.0 + exp(-x));
    else
        return exp(x) / (1.0 + exp(x));
}

void YOLOV5::save(const cv::Mat& img, const Armor& armor) const {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, ARMOR_NAMES[armor.name_], file_name);
    cv::imwrite(img_path, img);
}

//=== 调试用：画出并显示识别结果 ===
void YOLOV5::draw_detections(const cv::Mat& img, const std::list<Armor>& armors,
                             int frame_count) const {
    auto detection = img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto& armor : armors) {
        auto info = fmt::format("{:.2f} {} {} {}", armor.confidence_, COLORS[armor.color_],
                                ARMOR_NAMES[armor.name_], ARMOR_TYPES[armor.type_]);
        cv::Point2f armor_top = (armor.points_[0] + armor.points_[1]) / 2.0f;
        tools::draw_points(detection, armor.points_, {0, 255, 0});
        tools::draw_text(detection, info, armor_top, {0, 255, 0}, 0.7);
    }

    if (use_roi_) {
        cv::Scalar green(0, 255, 0);
        cv::rectangle(detection, roi_, green, 2);
    }
    // cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("yolov5_detection", detection);
}
//=== 保留接口 ===
std::list<Armor> YOLOV5::postprocess(double scale, cv::Mat& output, const cv::Mat& raw_img,
                                     int frame_count) {
    return parse(scale, output, raw_img, frame_count);
}

}  // namespace auto_aim
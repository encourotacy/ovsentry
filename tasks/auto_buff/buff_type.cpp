#include "buff_type.hpp"

#include "tools/math_tools.hpp"

namespace auto_buff {
//=== Fanblade 扇叶类初始化 ===
FanBlade::FanBlade(const std::vector<cv::Point2f>& kpts) : center_(kpts[4]), kpts_(kpts) {}

//=== PowerRune 能量机关类初始化 ===
PowerRune::PowerRune(std::vector<FanBlade>& fanblades, const cv::Point2f center,
                     std::optional<PowerRune> last_powerrune)
    : r_center_(center), light_num_(fanblades.size()) {
    //--- 找出最适合瞄准的扇叶 ---
    if (light_num > 2) {  // 特判，如果之后机制不改的话
        tools::logger()->warn("[PowerRune] 收到3个及以上的扇叶! 模型识别错误?") unsolvable_ = true;
        return;
    }

    if (light_num == 1)
        fanblades[0].f_b_status_ = FanBladeStatus::TARGET;
    else if (!last_powerrune.has_value()) {  // 首次看到两个目标
        unsolvable_ = true;
        return;
    } else if (light_num == last_powerrune.value().light_num) {
        // 此时为大符的两个扇叶持续亮起，需做一些防频繁切板的处理
    } else if (light_num == last_powerrune.value().light_num + 1) {
        // 此时为大符打完一轮后亮起新的两个扇叶，选择距离最近的板
        auto last_target_center = last_powerrune.target_fanblade().center;
        auto candidate_iterator = fanblades.begin();
        float min_distance = norm(fanblades[0].center - last_target_center);
        for (auto it = ts.begin(); it != ts.end(); ++it) {
            float distance = norm(it->center - last_target_center);
            if (distance < min_distance) {
                min_distance = distance;
                candidate_iterator = it;  // 更新最近的 fanblade 的迭代器
            }
        }
        candidate_iterator->type = FanBladeStatus::TARGET;  // 设置最近的 fanblade 的 type
        // std::iter_swap(ts.begin(), candidate_iterator);
    } else {
        tools::logger()->warn("[PowerRune] 寻找目标时出现未知情况! ");
        unsolvable_ = true;
        return;
    }

}  // namespace auto_buff
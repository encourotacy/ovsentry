#ifndef IO__ROS2_SUBSCRIBE_HPP
#define IO__ROS2_SUBSCRIBE_HPP

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace io {
enum class ROS2Mode : int { auto_aim = 0, small_buff = 1, big_buff = 2};

struct ArmorIgnoreList {
    bool enabled = false;
    std::vector<uint8_t> ignored_ids;

    bool contains(uint8_t id) const {
        if (!enabled || id == 0) return false;
        return std::find(ignored_ids.begin(), ignored_ids.end(), id) != ignored_ids.end();
    }
};

class ArmorIgnoreSubscriber {
   public:
    ArmorIgnoreSubscriber(const std::string& config_path);
    ~ArmorIgnoreSubscriber();

    ArmorIgnoreList ignored() const;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class StateSubscriber {
   public:
    explicit StateSubscriber(const std::string& config_path);
    ~StateSubscriber();
    
    ROS2Mode mode() const;

    std::string mode_name() const;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace io

#endif  // IO__ROS2_SUBSCRIBE_HPP
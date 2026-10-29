#ifndef IO__ROS2_SUBSCRIBE_HPP
#define IO__ROS2_SUBSCRIBE_HPP

#include <memory>
#include <string>

namespace io {
enum class ROS2Mode : int { auto_aim = 0, small_buff = 1, big_buff = 2};

class ArmorIgnoreSubscriber {
   public:
    ArmorIgnoreSubscriber(const std::string& config_path);
    ~ArmorIgnoreSubscriber();

    uint8_t ignore_ids() const;

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
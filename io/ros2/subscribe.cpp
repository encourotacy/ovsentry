#include "subscribe.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io {
//=== 初始化 ===
ArmorIgnoreSubscriber::ArmorIgnoreSubscriber(const std::string& config_path) {
    auto yaml = tools::load(config_path);
    auto subscribe = yaml["ros2_gimbal"]["subscribe"];

    auto topic = tools::read<std::string>(subscribe, "auto_aim_ignore_topic");
    auto msg_type = tools::read<std::string>(subscribe, "auto_aim_ignore_msg_type");

    impl_ = std::make_unique<ArmorIgnoreSubscriber::Impl>(topic, msg_type);
}

StateSubscriber::StateSubscriber(const std::string& config_path) {
    auto yaml = tools::load(config_path);
    auto subscribe = yaml["ros2_gimbal"]["subscribe"];
    auto topic = tools::read<std::string>(subscribe, "request_buff_topic");

    impl_ = std::make_unique<StateSubscriber::Impl>(topic);
}

ArmorIgnoreSubscriber::~ArmorIgnoreSubscriber() = default;
StateSubscriber::~StateSubscriber() = default;
//=== ArmorIgnore ===
class ArmorIgnoreSubscriber::Impl {
   public:
    Impl(const std::string& topic, const std::string& msg_type) {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
            self_initialized_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>("auto_aim_ignore_subscriber");
        try {
            subscription_ = node_->create_generic_subscription(
                topic, msg_type, rclcpp::SensorDataQoS(),
                [this](const std::shared_ptr<rclcpp::SerializedMessage> message) {
                    this->callback(message);
                });

            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_);
            spin_thread_ = std::thread([this]() { executor_->spin(); });
            tools::logger()->info("[AutoAimIgnore] Subscribed '{}' as '{}'.", topic, msg_type);
        } catch (const std::exception& e) {
            tools::logger()->warn("[AutoAimIgnore] Failed to subscribe '{}': {}", topic, e.what());
        }
    }

    ~Impl() {
        if (executor_) executor_->cancel();
        if (spin_thread_.joinable()) spin_thread_.join();
        if (executor_ && node_) executor_->remove_node(node_);
        if (self_initialized_ && rclcpp::ok()) rclcpp::shutdown();
    }

    size_t ignore_ids() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ignore_ids_;
    }

   private:
    void callback(const std::shared_ptr<rclcpp::SerializedMessage>& message) {
        try {
            const auto& raw = message->get_rcl_serialized_message();  // 读取原始字节
            eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char*>(raw.buffer),
                                                 raw.buffer_length);
            // FastBuffer 包装这段内容
            eprosima::fastcdr::Cdr cdr(buffer);  // 创建 CDR 读取器
            cdr.read_encapsulation();            // 读取 CDR 封装头

            // 读取数据
            uint8_t ignore_ids = 0;
            uint32_t count = 0;
            cdr >> count;
            for (uint32_t i = 0; i < count; ++i) {
                uint8_t id = 0;
                cdr >> id;
                if( id > 0 && id <= 8) ignore_ids += static_cast<uint8_t>(std::pow(2, --id));
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                ignore_ids_ = ignore_ids;
            }
        } catch (const std::exception& e) {
            tools::logger()->warn("[AutoAimIgnore] Failed to parse ignore ids: {}", e.what());
        }
    }

    mutable std::mutex mutex_;
    uint8_t ignore_ids_;
    bool self_initialized_ = false;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::GenericSubscription> subscription_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
};

uint8_t ArmorIgnoreSubscriber::ignore_ids() const { return impl_->ignore_ids(); }

//=== State ===
class StateSubscriber::Impl {
   public:
    explicit Impl(const std::string& topic) {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
            self_initialized_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>("buff_request_subscriber");
        subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
            topic, 10,
            [this](const std_msgs::msg::Int32::SharedPtr message) { this->callback(message); });

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });
        tools::logger()->info("[BuffRequest] Subscribed '{}' as Int32.", topic);
    }

    ~Impl() {
        if (executor_) executor_->cancel();
        if (spin_thread_.joinable()) spin_thread_.join();
        if (executor_ && node_) executor_->remove_node(node_);
        if (self_initialized_ && rclcpp::ok()) rclcpp::shutdown();
    }

    ROS2Mode mode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }

    std::string mode_name() const {
        std::lock_guard<std::mutex> lock(mutex_);
        switch (mode_) {
            case ROS2Mode::auto_aim:
                return "auto_aim";
            case ROS2Mode::small_buff:
                return "small_buff";
            case ROS2Mode::big_buff:
                return "big_buff";
        }
        return "unknown";
    }

   private:
    void callback(const std_msgs::msg::Int32::SharedPtr message) {
        ROS2Mode mode = ROS2Mode::auto_aim;
        switch (message->data) {
            case 0:
                mode = ROS2Mode::auto_aim;
                break;
            case 1:
                mode = ROS2Mode::small_buff;
                break;
            case 2:
                mode = ROS2Mode::big_buff;
                break;
            default:
                tools::logger()->warn("[StateRequest] ignore State={}", message->data);
                return ;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            mode_ = mode;
        }
    }

    mutable std::mutex mutex_;
    ROS2Mode mode_;
    bool self_initialized_ = false;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
};

ROS2Mode StateSubscriber::mode() const { return impl_->mode(); }

std::string StateSubscriber::mode_name() const { return impl_->mode_name(); }

}  // namespace io
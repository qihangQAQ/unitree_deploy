#include "Ros2DepthSource.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <limits>
#include <thread>

namespace sensors
{

struct Ros2DepthSource::Impl
{
    Impl(Ros2DepthSource* owner_, Ros2DepthSourceConfig cfg_)
        : owner(owner_), cfg(std::move(cfg_)) {}

    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        const bool is_u16 = msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
        const bool is_f32 = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
        if (!is_u16 && !is_f32) {
            owner->set_status("unsupported encoding: " + msg->encoding);
            if (!encoding_error_logged.exchange(true)) {
                spdlog::error(
                    "Ros2DepthSource: unsupported encoding '{}'; expected 16UC1 or 32FC1",
                    msg->encoding);
            }
            return;
        }

        const std::size_t bytes_per_pixel = is_u16 ? 2U : 4U;
        const std::size_t required_step = static_cast<std::size_t>(msg->width) * bytes_per_pixel;
        const std::size_t pixel_count =
            static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        if (msg->width == 0 || msg->height == 0 ||
            msg->width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            msg->height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
            pixel_count > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            msg->step < required_step ||
            msg->data.size() < static_cast<std::size_t>(msg->step) * msg->height) {
            owner->set_status("malformed ROS2 depth image");
            return;
        }

        DepthFrame frame;
        frame.width = static_cast<int>(msg->width);
        frame.height = static_cast<int>(msg->height);
        frame.source_encoding = msg->encoding;
        frame.frame_id = msg->header.frame_id;
        frame.sensor_timestamp_ns =
            static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(msg->header.stamp.nanosec);
        frame.received_at = SteadyClock::now();
        frame.sequence = ++sequence;
        frame.depth_m.resize(pixel_count);

        for (int v = 0; v < frame.height; ++v) {
            const std::uint8_t* row = msg->data.data() + static_cast<std::size_t>(v) * msg->step;
            for (int u = 0; u < frame.width; ++u) {
                float value = 0.0f;
                if (is_u16) {
                    const std::uint8_t* pixel = row + static_cast<std::size_t>(u) * 2U;
                    const std::uint16_t raw = msg->is_bigendian
                        ? static_cast<std::uint16_t>((pixel[0] << 8U) | pixel[1])
                        : static_cast<std::uint16_t>(pixel[0] | (pixel[1] << 8U));
                    value = static_cast<float>(raw) * 0.001f;
                } else {
                    const std::uint8_t* pixel = row + static_cast<std::size_t>(u) * 4U;
                    const std::uint32_t raw = msg->is_bigendian
                        ? (static_cast<std::uint32_t>(pixel[0]) << 24U) |
                          (static_cast<std::uint32_t>(pixel[1]) << 16U) |
                          (static_cast<std::uint32_t>(pixel[2]) << 8U) |
                          static_cast<std::uint32_t>(pixel[3])
                        : static_cast<std::uint32_t>(pixel[0]) |
                          (static_cast<std::uint32_t>(pixel[1]) << 8U) |
                          (static_cast<std::uint32_t>(pixel[2]) << 16U) |
                          (static_cast<std::uint32_t>(pixel[3]) << 24U);
                    std::memcpy(&value, &raw, sizeof(value));
                }
                frame.depth_m[static_cast<std::size_t>(v * frame.width + u)] = value;
            }
        }

        if (!first_frame_logged.exchange(true)) {
            spdlog::info(
                "Ros2DepthSource: first frame topic='{}' size={}x{} encoding={} step={}",
                cfg.image_topic, frame.width, frame.height, frame.source_encoding, msg->step);
        }

        const auto now = frame.received_at;
        if (last_rate_log.time_since_epoch().count() == 0) {
            last_rate_log = now;
            frames_since_rate_log = 0;
        }
        ++frames_since_rate_log;
        const double elapsed = std::chrono::duration<double>(now - last_rate_log).count();
        if (elapsed >= 5.0) {
            spdlog::info("Ros2DepthSource: receiving {:.1f} FPS", frames_since_rate_log / elapsed);
            last_rate_log = now;
            frames_since_rate_log = 0;
        }

        owner->publish(std::move(frame));
        owner->set_status("receiving " + msg->encoding);
    }

    void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        DepthCameraInfo info;
        info.width = static_cast<int>(msg->width);
        info.height = static_cast<int>(msg->height);
        std::copy(msg->k.begin(), msg->k.end(), info.k.begin());
        std::copy(msg->p.begin(), msg->p.end(), info.p.begin());
        info.frame_id = msg->header.frame_id;
        info.received_at = SteadyClock::now();
        owner->publish_camera_info(std::move(info));
        if (!camera_info_logged.exchange(true)) {
            spdlog::info(
                "Ros2DepthSource: camera info size={}x{} fx={:.4f} fy={:.4f} cx={:.4f} cy={:.4f}",
                msg->width, msg->height, msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
        }
    }

    Ros2DepthSource* owner;
    Ros2DepthSourceConfig cfg;
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription;
    std::thread thread;
    std::atomic_bool running{false};
    std::atomic_bool first_frame_logged{false};
    std::atomic_bool camera_info_logged{false};
    std::atomic_bool encoding_error_logged{false};
    std::uint64_t sequence = 0;
    SteadyClock::time_point last_rate_log{};
    std::uint64_t frames_since_rate_log = 0;
};

Ros2DepthSource::Ros2DepthSource(Ros2DepthSourceConfig cfg)
    : impl_(std::make_unique<Impl>(this, std::move(cfg))) {}

Ros2DepthSource::~Ros2DepthSource()
{
    stop();
}

bool Ros2DepthSource::start()
{
    if (impl_->running.exchange(true)) {
        return true;
    }
    try {
        impl_->context = std::make_shared<rclcpp::Context>();
        impl_->context->init(0, nullptr);

        rclcpp::NodeOptions node_options;
        node_options.context(impl_->context);
        impl_->node = std::make_shared<rclcpp::Node>(impl_->cfg.node_name, node_options);
        const auto qos = rclcpp::SensorDataQoS().keep_last(1);
        impl_->image_subscription = impl_->node->create_subscription<sensor_msgs::msg::Image>(
            impl_->cfg.image_topic,
            qos,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { impl_->on_image(msg); });
        impl_->camera_info_subscription = impl_->node->create_subscription<sensor_msgs::msg::CameraInfo>(
            impl_->cfg.camera_info_topic,
            qos,
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                impl_->on_camera_info(msg);
            });

        rclcpp::ExecutorOptions executor_options;
        executor_options.context = impl_->context;
        impl_->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        impl_->executor->add_node(impl_->node);
        impl_->thread = std::thread([this] {
            try {
                impl_->executor->spin();
            } catch (const std::exception& error) {
                if (impl_->running.exchange(false)) {
                    set_status(std::string("ROS2 executor stopped: ") + error.what());
                    spdlog::error("Ros2DepthSource: {}", status());
                }
            }
        });
        set_status("waiting for " + impl_->cfg.image_topic);
        spdlog::info(
            "Ros2DepthSource: subscribed image='{}' camera_info='{}'",
            impl_->cfg.image_topic, impl_->cfg.camera_info_topic);
        return true;
    } catch (const std::exception& error) {
        impl_->running.store(false);
        set_status(std::string("ROS2 startup failed: ") + error.what());
        spdlog::error("Ros2DepthSource: {}", status());
        return false;
    }
}

void Ros2DepthSource::stop()
{
    if (!impl_) {
        return;
    }
    const bool was_running = impl_->running.exchange(false);
    if (!was_running && !impl_->thread.joinable()) {
        return;
    }
    if (impl_->executor) {
        impl_->executor->cancel();
    }
    if (impl_->context && impl_->context->is_valid()) {
        impl_->context->shutdown("depth source stopped");
    }
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    impl_->camera_info_subscription.reset();
    impl_->image_subscription.reset();
    impl_->executor.reset();
    impl_->node.reset();
    impl_->context.reset();
    set_status("stopped");
}

} // namespace sensors

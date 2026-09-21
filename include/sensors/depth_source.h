#pragma once

#include "sensors/depth_frame.h"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace sensors
{

class DepthSource
{
public:
    virtual ~DepthSource() = default;

    virtual bool start() = 0;
    virtual void stop() = 0;

    bool ready() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<bool>(latest_);
    }

    std::shared_ptr<const DepthFrame> latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

    std::shared_ptr<const DepthFrame> wait_for_newer(
        std::uint64_t previous_sequence, std::chrono::milliseconds timeout) const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        frame_changed_.wait_for(lock, timeout, [&] {
            return latest_ && latest_->sequence != previous_sequence;
        });
        return latest_ && latest_->sequence != previous_sequence ? latest_ : nullptr;
    }

    std::optional<DepthCameraInfo> camera_info() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return camera_info_;
    }

    bool stale(std::chrono::milliseconds timeout) const
    {
        auto frame = latest();
        return !frame || SteadyClock::now() - frame->received_at > timeout;
    }

    std::string status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

protected:
    void publish(DepthFrame frame)
    {
        auto snapshot = std::make_shared<const DepthFrame>(std::move(frame));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(snapshot);
        }
        frame_changed_.notify_all();
    }

    void publish_camera_info(DepthCameraInfo info)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_info_ = std::move(info);
    }

    void set_status(std::string status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = std::move(status);
    }

private:
    mutable std::mutex mutex_;
    mutable std::condition_variable frame_changed_;
    std::shared_ptr<const DepthFrame> latest_;
    std::optional<DepthCameraInfo> camera_info_;
    std::string status_ = "not started";
};

} // namespace sensors

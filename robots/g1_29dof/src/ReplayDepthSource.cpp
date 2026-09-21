#include "sensors/replay_depth_source.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <spdlog/spdlog.h>

namespace sensors
{

ReplayDepthSource::ReplayDepthSource(ReplayDepthSourceConfig cfg) : cfg_(std::move(cfg)) {}

ReplayDepthSource::~ReplayDepthSource()
{
    stop();
}

bool ReplayDepthSource::load()
{
    if (cfg_.width <= 0 || cfg_.height <= 0 || !std::isfinite(cfg_.fps) || cfg_.fps <= 0.0f) {
        set_status("invalid replay dimensions or fps");
        return false;
    }

    std::ifstream stream(cfg_.file);
    if (!stream) {
        set_status("cannot open replay file: " + cfg_.file.string());
        return false;
    }

    std::vector<float> values;
    for (float value = 0.0f; stream >> value;) {
        values.push_back(value);
    }
    const std::size_t frame_size = static_cast<std::size_t>(cfg_.width * cfg_.height);
    if (values.empty() || values.size() % frame_size != 0) {
        set_status("replay value count is not a multiple of width*height");
        return false;
    }

    frames_.clear();
    for (std::size_t offset = 0; offset < values.size(); offset += frame_size) {
        frames_.emplace_back(values.begin() + offset, values.begin() + offset + frame_size);
    }
    return true;
}

bool ReplayDepthSource::start()
{
    if (running_.load()) {
        return true;
    }
    if (frames_.empty() && !load()) {
        spdlog::warn("ReplayDepthSource: {}", status());
        return false;
    }
    running_.store(true);
    set_status("running");
    thread_ = std::thread(&ReplayDepthSource::run, this);
    return true;
}

void ReplayDepthSource::stop()
{
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    set_status("stopped");
}

void ReplayDepthSource::run()
{
    using clock = SteadyClock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / cfg_.fps));
    auto next = clock::now();
    std::size_t index = 0;
    std::uint64_t sequence = 0;

    while (running_.load()) {
        DepthFrame frame;
        frame.depth_m = frames_[index];
        frame.width = cfg_.width;
        frame.height = cfg_.height;
        frame.source_encoding = "32FC1-replay";
        frame.received_at = clock::now();
        frame.sequence = ++sequence;
        publish(std::move(frame));

        ++index;
        if (index == frames_.size()) {
            if (!cfg_.loop) {
                running_.store(false);
                set_status("finished");
                break;
            }
            index = 0;
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
}

} // namespace sensors

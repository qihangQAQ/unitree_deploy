#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace sensors
{

using SteadyClock = std::chrono::steady_clock;

struct DepthFrame
{
    std::vector<float> depth_m;
    int width = 0;
    int height = 0;
    std::string source_encoding;
    std::string frame_id;
    std::int64_t sensor_timestamp_ns = 0;
    SteadyClock::time_point received_at = SteadyClock::now();
    std::uint64_t sequence = 0;

    bool valid() const
    {
        return width > 0 && height > 0 &&
               depth_m.size() == static_cast<std::size_t>(width * height);
    }
};

struct DepthCameraInfo
{
    int width = 0;
    int height = 0;
    std::array<double, 9> k{};
    std::array<double, 12> p{};
    std::string frame_id;
    SteadyClock::time_point received_at = SteadyClock::now();

    bool valid() const
    {
        return width > 0 && height > 0 && k[0] > 0.0 && k[4] > 0.0;
    }
};

struct ProcessedDepthFrame
{
    std::vector<float> data;
    int width = 0;
    int height = 0;
    std::size_t valid_count = 0;
    std::int64_t sensor_timestamp_ns = 0;
    SteadyClock::time_point received_at = SteadyClock::now();
    std::uint64_t sequence = 0;

    bool valid() const
    {
        return width > 0 && height > 0 &&
               data.size() == static_cast<std::size_t>(width * height);
    }
};

} // namespace sensors

#pragma once

#include "sensors/depth_frame.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sensors
{

struct DepthPreprocessorConfig
{
    int crop_x = 120;
    int crop_y = 110;
    int crop_width = 240;
    int crop_height = 160;
    int output_width = 24;
    int output_height = 16;
    float min_range_m = 0.3f;
    float max_range_m = 2.0f;
    float offset = -1.0f;

    // Training camera intrinsics after crop and resize.
    double target_fx = 24.1418;
    double target_fy = 24.1418;
    double target_cx = 12.25;
    double target_cy = 2.0;
    double intrinsics_relative_tolerance = 0.05;
};

struct TransformedIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

class DepthPreprocessor
{
public:
    explicit DepthPreprocessor(DepthPreprocessorConfig cfg = {}) : cfg_(std::move(cfg))
    {
        if (cfg_.crop_x < 0 || cfg_.crop_y < 0 || cfg_.crop_width <= 0 ||
            cfg_.crop_height <= 0 || cfg_.output_width <= 0 || cfg_.output_height <= 0) {
            throw std::invalid_argument("Invalid depth crop or output dimensions");
        }
        if (!(cfg_.min_range_m >= 0.0f && cfg_.max_range_m > cfg_.min_range_m)) {
            throw std::invalid_argument("Invalid depth range");
        }
        if (!std::isfinite(cfg_.offset) || cfg_.intrinsics_relative_tolerance < 0.0 ||
            !std::isfinite(cfg_.target_fx) || !std::isfinite(cfg_.target_fy) ||
            !std::isfinite(cfg_.target_cx) || !std::isfinite(cfg_.target_cy) ||
            cfg_.target_fx <= 0.0 || cfg_.target_fy <= 0.0) {
            throw std::invalid_argument("Invalid depth normalization or target intrinsics");
        }
    }

    ProcessedDepthFrame process(const DepthFrame& frame) const
    {
        if (!frame.valid()) {
            throw std::runtime_error("Invalid source depth frame");
        }
        if (cfg_.crop_x + cfg_.crop_width > frame.width ||
            cfg_.crop_y + cfg_.crop_height > frame.height) {
            throw std::runtime_error(
                "Depth frame is smaller than configured crop: frame=" +
                std::to_string(frame.width) + "x" + std::to_string(frame.height) +
                ", crop=" + std::to_string(cfg_.crop_x) + "," +
                std::to_string(cfg_.crop_y) + "," +
                std::to_string(cfg_.crop_width) + "," +
                std::to_string(cfg_.crop_height));
        }

        ProcessedDepthFrame output;
        output.width = cfg_.output_width;
        output.height = cfg_.output_height;
        output.sensor_timestamp_ns = frame.sensor_timestamp_ns;
        output.received_at = frame.received_at;
        output.sequence = frame.sequence;
        output.data.resize(static_cast<std::size_t>(output.width * output.height));

        for (int v = 0; v < output.height; ++v) {
            const int source_v = cfg_.crop_y + (v * cfg_.crop_height) / output.height;
            for (int u = 0; u < output.width; ++u) {
                const int source_u = cfg_.crop_x + (u * cfg_.crop_width) / output.width;
                const float depth = frame.depth_m[
                    static_cast<std::size_t>(source_v * frame.width + source_u)];
                const bool valid = std::isfinite(depth) &&
                                   depth >= cfg_.min_range_m &&
                                   depth <= cfg_.max_range_m;
                auto& value = output.data[static_cast<std::size_t>(v * output.width + u)];
                if (valid) {
                    value = depth + cfg_.offset;
                    ++output.valid_count;
                } else {
                    value = cfg_.offset;
                }
            }
        }
        return output;
    }

    TransformedIntrinsics transform_intrinsics(const DepthCameraInfo& info) const
    {
        if (!info.valid()) {
            throw std::runtime_error("Invalid depth camera info");
        }
        const double scale_x = static_cast<double>(cfg_.output_width) / cfg_.crop_width;
        const double scale_y = static_cast<double>(cfg_.output_height) / cfg_.crop_height;
        return {
            info.k[0] * scale_x,
            info.k[4] * scale_y,
            (info.k[2] - cfg_.crop_x) * scale_x,
            (info.k[5] - cfg_.crop_y) * scale_y,
        };
    }

    bool intrinsics_match(const DepthCameraInfo& info, std::string* reason = nullptr) const
    {
        const auto actual = transform_intrinsics(info);
        const auto close = [this](double value, double expected) {
            const double scale = std::max(1.0, std::fabs(expected));
            return std::fabs(value - expected) <= cfg_.intrinsics_relative_tolerance * scale;
        };
        const bool matches = close(actual.fx, cfg_.target_fx) &&
                             close(actual.fy, cfg_.target_fy) &&
                             close(actual.cx, cfg_.target_cx) &&
                             close(actual.cy, cfg_.target_cy);
        if (!matches && reason) {
            *reason = "transformed intrinsics actual=[" + std::to_string(actual.fx) + "," +
                      std::to_string(actual.fy) + "," + std::to_string(actual.cx) + "," +
                      std::to_string(actual.cy) + "] expected=[" +
                      std::to_string(cfg_.target_fx) + "," + std::to_string(cfg_.target_fy) + "," +
                      std::to_string(cfg_.target_cx) + "," + std::to_string(cfg_.target_cy) + "]";
        }
        return matches;
    }

    const DepthPreprocessorConfig& config() const { return cfg_; }
    std::size_t output_elements() const
    {
        return static_cast<std::size_t>(cfg_.output_width * cfg_.output_height);
    }

private:
    DepthPreprocessorConfig cfg_;
};

} // namespace sensors

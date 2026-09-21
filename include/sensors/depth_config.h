#pragma once

#include "sensors/depth_preprocessor.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <vector>

namespace sensors
{

inline DepthPreprocessorConfig load_depth_preprocessor_config(const YAML::Node& cfg)
{
    DepthPreprocessorConfig result;
    if (cfg && cfg["crop"]) {
        const auto crop = cfg["crop"].as<std::vector<int>>();
        if (crop.size() != 4) {
            throw std::runtime_error("depth.crop must be [x, y, width, height]");
        }
        result.crop_x = crop[0];
        result.crop_y = crop[1];
        result.crop_width = crop[2];
        result.crop_height = crop[3];
    }
    if (cfg && cfg["output_size"]) {
        const auto size = cfg["output_size"].as<std::vector<int>>();
        if (size.size() != 2) {
            throw std::runtime_error("depth.output_size must be [width, height]");
        }
        result.output_width = size[0];
        result.output_height = size[1];
    }
    if (cfg && cfg["min_range"]) {
        result.min_range_m = cfg["min_range"].as<float>();
    }
    if (cfg && cfg["max_range"]) {
        result.max_range_m = cfg["max_range"].as<float>();
    }
    if (cfg && cfg["offset"]) {
        result.offset = cfg["offset"].as<float>();
    }
    if (cfg && cfg["intrinsics_relative_tolerance"]) {
        result.intrinsics_relative_tolerance = cfg["intrinsics_relative_tolerance"].as<double>();
    }
    if (cfg && cfg["target_intrinsics"]) {
        const auto intrinsics = cfg["target_intrinsics"].as<std::vector<double>>();
        if (intrinsics.size() != 4) {
            throw std::runtime_error("depth.target_intrinsics must be [fx, fy, cx, cy]");
        }
        result.target_fx = intrinsics[0];
        result.target_fy = intrinsics[1];
        result.target_cx = intrinsics[2];
        result.target_cy = intrinsics[3];
    }
    return result;
}

} // namespace sensors

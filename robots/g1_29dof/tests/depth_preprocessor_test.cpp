#include "sensors/depth_preprocessor.h"
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/manager_term_cfg.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}
}

int main()
{
    sensors::DepthPreprocessorConfig cfg;
    cfg.crop_x = 1;
    cfg.crop_y = 1;
    cfg.crop_width = 4;
    cfg.crop_height = 2;
    cfg.output_width = 2;
    cfg.output_height = 1;
    cfg.min_range_m = 0.3f;
    cfg.max_range_m = 2.0f;
    cfg.offset = -1.0f;

    sensors::DepthFrame frame;
    frame.width = 6;
    frame.height = 4;
    frame.depth_m.assign(24, 1.0f);
    frame.depth_m[1 * frame.width + 1] = 0.2f;
    frame.depth_m[1 * frame.width + 3] = 1.5f;
    frame.source_encoding = "test";

    sensors::DepthPreprocessor preprocessor(cfg);
    const auto output = preprocessor.process(frame);
    require(output.width == 2, "wrong output width");
    require(output.height == 1, "wrong output height");
    require(output.data.size() == 2, "wrong output element count");
    require(output.valid_count == 1, "wrong valid-pixel count");
    require(output.data[0] == -1.0f, "invalid depth was not normalized to -1");
    require(std::fabs(output.data[1] - 0.5f) < 1e-6f, "valid depth normalization mismatch");

    frame.depth_m[1 * frame.width + 3] = std::numeric_limits<float>::quiet_NaN();
    const auto invalid_output = preprocessor.process(frame);
    require(invalid_output.valid_count == 0, "NaN depth counted as valid");
    require(invalid_output.data[0] == -1.0f, "low depth normalization mismatch");
    require(invalid_output.data[1] == -1.0f, "NaN depth normalization mismatch");

    sensors::DepthCameraInfo info;
    info.width = 480;
    info.height = 270;
    info.k = {241.418, 0.0, 242.5, 0.0, 241.418, 130.0, 0.0, 0.0, 1.0};
    sensors::DepthPreprocessorConfig camera_cfg;
    sensors::DepthPreprocessor camera_preprocessor(camera_cfg);
    const auto intrinsics = camera_preprocessor.transform_intrinsics(info);
    require(std::fabs(intrinsics.fx - 24.1418) < 1e-6, "transformed fx mismatch");
    require(std::fabs(intrinsics.fy - 24.1418) < 1e-6, "transformed fy mismatch");
    require(std::fabs(intrinsics.cx - 12.25) < 1e-6, "transformed cx mismatch");
    require(std::fabs(intrinsics.cy - 2.0) < 1e-6, "transformed cy mismatch");
    require(camera_preprocessor.intrinsics_match(info), "matching intrinsics were rejected");

    isaaclab::ObservationTermCfg observation_term;
    observation_term.scale.assign(16, 1.0f);
    bool rejected_bad_scale = false;
    try {
        observation_term.add(std::vector<float>(384, 0.0f));
    } catch (const std::runtime_error&) {
        rejected_bad_scale = true;
    }
    require(rejected_bad_scale, "bad observation scale length was accepted");

    bool rejected_bad_config = false;
    try {
        sensors::DepthPreprocessorConfig bad_cfg;
        bad_cfg.intrinsics_relative_tolerance = -1.0;
        sensors::DepthPreprocessor invalid_preprocessor(bad_cfg);
        (void)invalid_preprocessor;
    } catch (const std::invalid_argument&) {
        rejected_bad_config = true;
    }
    require(rejected_bad_config, "invalid preprocessor config was accepted");

    std::cout << "depth_preprocessor_test passed\n";
    return 0;
}

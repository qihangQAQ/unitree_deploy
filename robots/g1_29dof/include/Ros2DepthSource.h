#pragma once

#include "sensors/depth_source.h"

#include <memory>
#include <string>

namespace sensors
{

struct Ros2DepthSourceConfig
{
    std::string node_name = "unitree_depth_source";
    std::string image_topic = "/camera/depth/image_rect_raw";
    std::string camera_info_topic = "/camera/depth/camera_info";
};

class Ros2DepthSource : public DepthSource
{
public:
    explicit Ros2DepthSource(Ros2DepthSourceConfig cfg);
    ~Ros2DepthSource() override;

    bool start() override;
    void stop() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sensors

#pragma once

#include "sensors/depth_source.h"

#include <atomic>
#include <filesystem>
#include <thread>
#include <vector>

namespace sensors
{

struct ReplayDepthSourceConfig
{
    std::filesystem::path file;
    int width = 0;
    int height = 0;
    float fps = 60.0f;
    bool loop = true;
};

class ReplayDepthSource : public DepthSource
{
public:
    explicit ReplayDepthSource(ReplayDepthSourceConfig cfg);
    ~ReplayDepthSource() override;

    bool start() override;
    void stop() override;

private:
    bool load();
    void run();

    ReplayDepthSourceConfig cfg_;
    std::vector<std::vector<float>> frames_;
    std::thread thread_;
    std::atomic_bool running_{false};
};

} // namespace sensors

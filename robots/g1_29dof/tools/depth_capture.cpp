#include "Ros2DepthSource.h"
#include "sensors/depth_config.h"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <unistd.h>

namespace
{

namespace fs = std::filesystem;
volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int)
{
    stop_requested = 1;
}

std::string topic_from_config(const YAML::Node& cfg, const char* key,
                              const char* environment, const char* fallback)
{
    if (const char* value = std::getenv(environment); value && *value) {
        return value;
    }
    return cfg && cfg[key] ? cfg[key].as<std::string>() : fallback;
}

void write_frame_header(std::ostream& out, const sensors::DepthFrame& frame)
{
    out << "# sequence=" << frame.sequence << '\n'
        << "# sensor_timestamp_ns=" << frame.sensor_timestamp_ns << '\n'
        << "# source_encoding=" << frame.source_encoding << '\n'
        << "# frame_id=" << frame.frame_id << '\n'
        << "# raw_size=" << frame.width << ',' << frame.height << '\n';
}

template <std::size_t N>
void write_intrinsics(std::ostream& out, const char* label, const std::array<double, N>& values)
{
    out << "# " << label << '=';
    for (std::size_t i = 0; i < N; ++i) {
        if (i != 0) {
            out << ',';
        }
        out << values[i];
    }
    out << '\n';
}

void write_rows(std::ostream& out, const std::vector<float>& data, int width, int height)
{
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            if (column != 0) {
                out << ',';
            }
            out << data[static_cast<std::size_t>(row * width + column)];
        }
        out << '\n';
    }
}

template <typename Writer>
void atomic_write(const fs::path& destination, Writer&& writer)
{
    fs::path temporary = destination;
    temporary += ".tmp";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out) {
            throw std::runtime_error("Cannot open " + temporary.string());
        }
        out << std::setprecision(std::numeric_limits<float>::max_digits10);
        writer(out);
        out.flush();
        if (!out) {
            throw std::runtime_error("Cannot write " + temporary.string());
        }
        out.close();
        if (!out) {
            throw std::runtime_error("Cannot close " + temporary.string());
        }
    }
    fs::rename(temporary, destination);
}

void save_processed(const fs::path& destination, const sensors::DepthFrame& frame,
                    const sensors::DepthPreprocessor& preprocessor,
                    const std::optional<sensors::DepthCameraInfo>& info)
{
    const auto processed = preprocessor.process(frame);
    const auto& cfg = preprocessor.config();
    atomic_write(destination, [&](std::ostream& out) {
        out << "# format=unitree_policy_depth_v1\n";
        write_frame_header(out, frame);
        out << "# size=" << processed.width << ',' << processed.height << '\n'
            << "# crop=" << cfg.crop_x << ',' << cfg.crop_y << ','
            << cfg.crop_width << ',' << cfg.crop_height << '\n'
            << "# valid_range_m=" << cfg.min_range_m << ',' << cfg.max_range_m << '\n'
            << "# offset=" << cfg.offset << '\n'
            << "# valid_count=" << processed.valid_count << '\n';
        if (info) {
            out << "# camera_info_size=" << info->width << ',' << info->height << '\n'
                << "# camera_info_frame_id=" << info->frame_id << '\n';
            write_intrinsics(out, "camera_info_k", info->k);
            write_intrinsics(out, "camera_info_p", info->p);
            if (info->valid() && info->width == frame.width && info->height == frame.height) {
                const auto transformed = preprocessor.transform_intrinsics(*info);
                std::string mismatch;
                const bool matches = preprocessor.intrinsics_match(*info, &mismatch);
                out << "# transformed_k=" << transformed.fx << ',' << transformed.fy << ','
                    << transformed.cx << ',' << transformed.cy << '\n'
                    << "# target_intrinsics_match=" << (matches ? "true" : "false") << '\n';
                if (!matches) {
                    out << "# intrinsics_mismatch=" << mismatch << '\n';
                }
            } else {
                out << "# target_intrinsics_match=unavailable\n";
            }
        } else {
            out << "# camera_info=missing\n";
        }
        write_rows(out, processed.data, processed.width, processed.height);
    });
}

void save_raw(const fs::path& destination, const sensors::DepthFrame& frame)
{
    atomic_write(destination, [&](std::ostream& out) {
        out << "# format=unitree_raw_depth_m_v1\n";
        write_frame_header(out, frame);
        out << "# size=" << frame.width << ',' << frame.height << '\n';
        write_rows(out, frame.depth_m, frame.width, frame.height);
    });
}

std::string local_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    if (!localtime_r(&seconds, &local_time)) {
        throw std::runtime_error("Cannot read local time");
    }
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream name;
    name << std::put_time(&local_time, "%Y%m%d_%H%M%S")
         << '_' << std::setw(3) << std::setfill('0') << milliseconds;
    return name.str();
}

std::pair<fs::path, fs::path> final_paths(const fs::path& directory)
{
    const std::string timestamp = local_timestamp();
    for (int suffix = 0; suffix < 1000; ++suffix) {
        const std::string stem = timestamp + (suffix == 0 ? "" : "_" + std::to_string(suffix));
        const auto processed = directory / (stem + "_depth-real.csv");
        const auto raw = directory / (stem + "_depth-real-raw.csv");
        if (!fs::exists(processed) && !fs::exists(raw)) {
            return {processed, raw};
        }
    }
    throw std::runtime_error("Cannot find an unused timestamped depth filename");
}

int run()
{
    // In either layout, build/ or bin/ sits beside config/ under one parent.
    const fs::path robot_directory = fs::canonical("/proc/self/exe").parent_path().parent_path();
    const fs::path config_path = robot_directory / "config" / "config.yaml";
    const fs::path output_directory = robot_directory / "depth";
    if (!fs::is_regular_file(config_path)) {
        throw std::runtime_error("Depth config not found: " + config_path.string());
    }
    const YAML::Node root = YAML::LoadFile(config_path.string());
    const YAML::Node depth_cfg = root["depth"];
    if (!depth_cfg) {
        throw std::runtime_error("Config file has no depth section");
    }

    const sensors::DepthPreprocessor preprocessor(
        sensors::load_depth_preprocessor_config(depth_cfg));
    sensors::Ros2DepthSourceConfig source_cfg;
    source_cfg.node_name = "unitree_depth_capture";
    source_cfg.image_topic = topic_from_config(
        depth_cfg, "topic", "UNITREE_DEPTH_TOPIC", "/camera/depth/image_rect_raw");
    source_cfg.camera_info_topic = topic_from_config(
        depth_cfg, "camera_info_topic", "UNITREE_DEPTH_CAMERA_INFO_TOPIC",
        "/camera/depth/camera_info");
    const int stale_timeout_ms = depth_cfg["capture_stale_timeout_ms"]
        ? depth_cfg["capture_stale_timeout_ms"].as<int>() : 500;
    if (stale_timeout_ms <= 0) {
        throw std::runtime_error("depth.capture_stale_timeout_ms must be positive");
    }
    const auto stale_timeout = std::chrono::milliseconds(stale_timeout_ms);

    if (fs::exists(output_directory) && !fs::is_directory(output_directory)) {
        throw std::runtime_error("Depth output path is not a directory: " + output_directory.string());
    }
    fs::create_directories(output_directory);
    const fs::path staging_file = output_directory /
        (".depth_capture_" + std::to_string(static_cast<long>(getpid())) + ".csv");

    // Keep this standalone tool usable when the default ~/.ros/log is not writable.
    if (const char* log_directory = std::getenv("ROS_LOG_DIR"); !log_directory || !*log_directory) {
        const auto local_log_directory = fs::absolute(output_directory / "ros_logs");
        fs::create_directories(local_log_directory);
        if (setenv("ROS_LOG_DIR", local_log_directory.c_str(), 1) != 0) {
            throw std::runtime_error("Cannot set ROS_LOG_DIR");
        }
    }

    sensors::Ros2DepthSource source(std::move(source_cfg));
    if (!source.start()) {
        throw std::runtime_error(source.status());
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "Capturing depth into " << output_directory << '\n'
              << "The latest processed frame is updated on disk during capture.\n"
              << "Stop the camera node or press Ctrl+C here to finish.\n" << std::flush;

    std::uint64_t saved_sequence = 0;
    const auto first_frame_deadline = sensors::SteadyClock::now() + std::chrono::seconds(15);
    std::exception_ptr capture_error;
    try {
        while (!stop_requested) {
            const auto frame = source.wait_for_newer(saved_sequence, std::chrono::milliseconds(100));
            if (frame) {
                save_processed(staging_file, *frame, preprocessor, source.camera_info());
                saved_sequence = frame->sequence;
                if (saved_sequence == 1 || saved_sequence % 60 == 0) {
                    std::cout << "Saved sequence " << saved_sequence << '\n' << std::flush;
                }
            }
            if (saved_sequence != 0 && source.stale(stale_timeout)) {
                std::cout << "Camera stream stopped; saving final raw frame.\n";
                break;
            }
            if (saved_sequence == 0 && sensors::SteadyClock::now() >= first_frame_deadline) {
                throw std::runtime_error("No depth frame received within 15 seconds: " + source.status());
            }
        }
    } catch (...) {
        capture_error = std::current_exception();
    }

    // Stop the subscriber before reading its final snapshot: no frame can arrive after this point.
    source.stop();
    const auto final_frame = source.latest();
    if (final_frame) {
        if (!capture_error && final_frame->sequence != saved_sequence) {
            try {
                save_processed(staging_file, *final_frame, preprocessor, source.camera_info());
            } catch (...) {
                capture_error = std::current_exception();
            }
        }
        const auto [processed_file, raw_file] = final_paths(output_directory);
        if (!capture_error) {
            try {
                fs::rename(staging_file, processed_file);
                std::cout << "Final policy depth saved to " << processed_file << '\n';
            } catch (...) {
                capture_error = std::current_exception();
            }
        }
        try {
            save_raw(raw_file, *final_frame);
            std::cout << "Final raw depth saved to " << raw_file << '\n';
        } catch (...) {
            if (!capture_error) {
                capture_error = std::current_exception();
            }
        }
    } else if (!capture_error) {
        capture_error = std::make_exception_ptr(
            std::runtime_error("Stopped before receiving a depth frame"));
    }
    if (capture_error) {
        std::rethrow_exception(capture_error);
    }
    std::cout << "Final sequence " << final_frame->sequence << " saved\n";
    return 0;
}

} // namespace

int main(int argc, char**)
{
    if (argc != 1) {
        std::cerr << "Usage: depth_capture\n";
        return 2;
    }
    try {
        return run();
    } catch (const std::exception& error) {
        std::cerr << "depth_capture: " << error.what() << '\n';
        return 1;
    }
}

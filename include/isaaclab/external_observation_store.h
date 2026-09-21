#pragma once

#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

class ExternalObservationStore
{
public:
    void set(const std::string& name, std::vector<float> value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        values_[name] = std::move(value);
    }

    std::vector<float> get(const std::string& name) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = values_.find(name);
        if (it == values_.end()) {
            throw std::runtime_error("External observation '" + name + "' is not available");
        }
        return it->second;
    }

    bool contains(const std::string& name) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return values_.find(name) != values_.end();
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::vector<float>> values_;
};

} // namespace isaaclab

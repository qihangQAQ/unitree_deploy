// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

using ObservationMap = std::unordered_map<std::string, std::vector<float>>;

class Algorithms
{
public:
    virtual ~Algorithms() = default;
    virtual std::vector<float> act(const ObservationMap& obs) = 0;

    std::vector<float> get_action() const
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action_;
    }

protected:
    mutable std::mutex act_mtx_;
    std::vector<float> action_;
};

struct TensorContract
{
    std::string name;
    std::vector<std::int64_t> shape;
    ONNXTensorElementDataType element_type = ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED;
    std::size_t element_count = 0;
};

class OrtRunner : public Algorithms
{
public:
    explicit OrtRunner(const std::filesystem::path& model_path)
        : env_(ORT_LOGGING_LEVEL_WARNING, "unitree_deploy"), model_path_(model_path)
    {
        if (!std::filesystem::is_regular_file(model_path_)) {
            throw std::runtime_error("ONNX model does not exist: " + model_path_.string());
        }

        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(
            env_, model_path_.string().c_str(), session_options_);

        const std::size_t input_count = session_->GetInputCount();
        if (input_count == 0) {
            throw std::runtime_error("ONNX model has no inputs");
        }
        for (std::size_t i = 0; i < input_count; ++i) {
            inputs_.push_back(read_tensor_contract(true, i));
        }

        if (session_->GetOutputCount() != 1) {
            throw std::runtime_error("ONNX model must have exactly one output");
        }
        output_ = read_tensor_contract(false, 0);
        if (output_.element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            throw std::runtime_error("ONNX output must be float32");
        }
        action_.assign(output_.element_count, 0.0f);
    }

    std::vector<float> act(const ObservationMap& obs) override
    {
        const auto started_at = std::chrono::steady_clock::now();
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        std::vector<Ort::Value> input_tensors;
        std::vector<const char*> input_names;
        input_tensors.reserve(inputs_.size());
        input_names.reserve(inputs_.size());

        for (const auto& input : inputs_) {
            const auto found = obs.find(input.name);
            if (found == obs.end()) {
                throw std::runtime_error("ONNX input '" + input.name + "' is missing from observations");
            }
            const auto& input_data = found->second;
            if (input_data.size() != input.element_count) {
                throw std::runtime_error(
                    "ONNX input '" + input.name + "' expects " +
                    std::to_string(input.element_count) + " values, got " +
                    std::to_string(input_data.size()));
            }
            if (!std::all_of(input_data.begin(), input_data.end(),
                             [](float value) { return std::isfinite(value); })) {
                throw std::runtime_error("ONNX input '" + input.name + "' contains NaN or Inf");
            }
            if (input.element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                throw std::runtime_error("ONNX input '" + input.name + "' is not float32");
            }

            // ONNX Runtime does not mutate inputs, but its C++ API takes a mutable pointer.
            input_tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info,
                const_cast<float*>(input_data.data()),
                input_data.size(),
                input.shape.data(),
                input.shape.size()));
            input_names.push_back(input.name.c_str());
        }

        const char* output_name = output_.name.c_str();
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            input_tensors.data(),
            input_tensors.size(),
            &output_name,
            1);
        if (output_tensors.size() != 1 || !output_tensors.front().IsTensor()) {
            throw std::runtime_error("ONNX Runtime returned an invalid output");
        }

        const auto runtime_info = output_tensors.front().GetTensorTypeAndShapeInfo();
        if (runtime_info.GetElementCount() != output_.element_count) {
            throw std::runtime_error("ONNX Runtime output size changed unexpectedly");
        }
        const float* values = output_tensors.front().GetTensorData<float>();
        std::vector<float> result(values, values + output_.element_count);
        if (!std::all_of(result.begin(), result.end(),
                         [](float value) { return std::isfinite(value); })) {
            throw std::runtime_error("ONNX output contains NaN or Inf");
        }

        {
            std::lock_guard<std::mutex> lock(act_mtx_);
            action_ = result;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - started_at);
        last_inference_us_.store(elapsed.count());
        return result;
    }

    const std::vector<TensorContract>& inputs() const { return inputs_; }
    const TensorContract& output() const { return output_; }
    std::int64_t last_inference_us() const { return last_inference_us_.load(); }

    const TensorContract* find_input(const std::string& name) const
    {
        const auto it = std::find_if(inputs_.begin(), inputs_.end(),
            [&name](const TensorContract& input) { return input.name == name; });
        return it == inputs_.end() ? nullptr : &*it;
    }

private:
    TensorContract read_tensor_contract(bool input, std::size_t index)
    {
        Ort::TypeInfo type_info = input
            ? session_->GetInputTypeInfo(index)
            : session_->GetOutputTypeInfo(index);
        const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        TensorContract contract;
        contract.shape = tensor_info.GetShape();
        contract.element_type = tensor_info.GetElementType();
        if (contract.shape.empty()) {
            throw std::runtime_error("ONNX scalar tensors are not supported");
        }
        contract.element_count = 1;
        for (std::int64_t dim : contract.shape) {
            if (dim <= 0) {
                throw std::runtime_error("ONNX model must use fixed positive tensor dimensions");
            }
            contract.element_count *= static_cast<std::size_t>(dim);
        }
        auto allocated_name = input
            ? session_->GetInputNameAllocated(index, allocator_)
            : session_->GetOutputNameAllocated(index, allocator_);
        contract.name = allocated_name.get();
        return contract;
    }

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::filesystem::path model_path_;
    std::vector<TensorContract> inputs_;
    TensorContract output_;
    std::atomic<std::int64_t> last_inference_us_{0};
};

} // namespace isaaclab

#include "robosim/ai/onnx_inference.h"
#include <spdlog/spdlog.h>

#ifdef ROBOSIM_HAS_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace robosim::ai {

struct OnnxInference::Impl {
#ifdef ROBOSIM_HAS_ONNX
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "robosim"};
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<const char*> input_name_ptrs;
    std::vector<const char*> output_name_ptrs;
#endif
};

OnnxInference::~OnnxInference() = default;

bool OnnxInference::load_model(const std::string& model_path) {
#ifdef ROBOSIM_HAS_ONNX
    try {
        impl_ = std::make_unique<Impl>();
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        impl_->session = std::make_unique<Ort::Session>(
            impl_->env, model_path.c_str(), options);

        // Get input info
        Ort::AllocatorWithDefaultOptions allocator;
        size_t num_inputs = impl_->session->GetInputCount();
        for (size_t i = 0; i < num_inputs; i++) {
            auto name = impl_->session->GetInputNameAllocated(i, allocator);
            impl_->input_names.push_back(name.get());
        }
        for (auto& n : impl_->input_names) impl_->input_name_ptrs.push_back(n.c_str());

        // Get first input shape for dim
        auto type_info = impl_->session->GetInputTypeInfo(0);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        auto shape = tensor_info.GetShape();
        input_dim_ = (shape.size() > 1) ? static_cast<int>(shape[1]) : static_cast<int>(shape[0]);

        // Get output info
        size_t num_outputs = impl_->session->GetOutputCount();
        for (size_t i = 0; i < num_outputs; i++) {
            auto name = impl_->session->GetOutputNameAllocated(i, allocator);
            impl_->output_names.push_back(name.get());
        }
        for (auto& n : impl_->output_names) impl_->output_name_ptrs.push_back(n.c_str());

        auto out_type_info = impl_->session->GetOutputTypeInfo(0);
        auto out_tensor_info = out_type_info.GetTensorTypeAndShapeInfo();
        auto out_shape = out_tensor_info.GetShape();
        output_dim_ = (out_shape.size() > 1) ? static_cast<int>(out_shape[1]) : static_cast<int>(out_shape[0]);

        model_path_ = model_path;
        loaded_ = true;
        spdlog::info("ONNX model loaded: {} (input_dim={}, output_dim={})",
                     model_path, input_dim_, output_dim_);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to load ONNX model: {}", e.what());
        return false;
    }
#else
    spdlog::warn("ONNX Runtime not available. Build with -DROBOSIM_USE_ONNX=ON");
    return false;
#endif
}

Eigen::VectorXd OnnxInference::infer(const Eigen::VectorXd& input) const {
#ifdef ROBOSIM_HAS_ONNX
    if (!loaded_ || !impl_->session) return Eigen::VectorXd();

    std::vector<float> input_data(input.size());
    for (int i = 0; i < input.size(); i++) input_data[i] = static_cast<float>(input(i));

    std::vector<int64_t> input_shape = {1, static_cast<int64_t>(input.size())};
    auto input_tensor = Ort::Value::CreateTensor<float>(
        impl_->memory_info, input_data.data(), input_data.size(),
        input_shape.data(), input_shape.size());

    auto output_tensors = impl_->session->Run(
        Ort::RunOptions{nullptr},
        impl_->input_name_ptrs.data(), &input_tensor, 1,
        impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    auto out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    int out_size = (out_shape.size() > 1) ? static_cast<int>(out_shape[1]) : static_cast<int>(out_shape[0]);

    Eigen::VectorXd result(out_size);
    for (int i = 0; i < out_size; i++) result(i) = static_cast<double>(output_data[i]);
    return result;
#else
    return Eigen::VectorXd::Zero(0);
#endif
}

} // namespace robosim::ai

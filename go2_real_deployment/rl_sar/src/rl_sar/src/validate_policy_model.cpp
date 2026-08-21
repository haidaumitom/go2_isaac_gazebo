#include "inference_runtime.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int RunValidation(int argc, char** argv)
{
    if (argc < 4 || argc > 6)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <model_path> <single|flat_history|obs_history> <observation_dim>"
                  << " [history_length] [action_dim]" << std::endl;
        return 2;
    }

    const std::string model_path = argv[1];
    const std::string mode = argv[2];
    const int64_t observation_dim = std::stoll(argv[3]);
    const int64_t history_length = argc >= 5 ? std::stoll(argv[4]) : 0;
    const size_t action_dim = argc >= 6 ? static_cast<size_t>(std::stoll(argv[5])) : 12;

    if (observation_dim <= 0 || action_dim == 0)
    {
        throw std::invalid_argument("Observation and action dimensions must be positive");
    }
    if ((mode == "flat_history" || mode == "obs_history") && history_length <= 0)
    {
        throw std::invalid_argument("History modes require a positive history length");
    }

    std::vector<std::vector<float>> inputs;
    std::vector<std::vector<int64_t>> shapes;
    if (mode == "single")
    {
        inputs = {std::vector<float>(observation_dim, 0.0f)};
        shapes = {{1, observation_dim}};
    }
    else if (mode == "flat_history")
    {
        inputs = {std::vector<float>(history_length * observation_dim, 0.0f)};
        shapes = {{1, history_length * observation_dim}};
    }
    else if (mode == "obs_history")
    {
        inputs = {
            std::vector<float>(observation_dim, 0.0f),
            std::vector<float>(history_length * observation_dim, 0.0f)
        };
        shapes = {{1, observation_dim}, {1, history_length, observation_dim}};
    }
    else
    {
        throw std::invalid_argument("Unsupported model mode: " + mode);
    }

    auto model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model)
    {
        throw std::runtime_error("Failed to load model: " + model_path);
    }

    const auto actions = model->forward_with_shapes(inputs, shapes);
    if (actions.size() != action_dim)
    {
        throw std::runtime_error(
            "Model returned " + std::to_string(actions.size()) + " actions; expected " + std::to_string(action_dim));
    }
    for (float action : actions)
    {
        if (!std::isfinite(action))
        {
            throw std::runtime_error("Model returned a non-finite action");
        }
    }

    std::cout << "Policy model validation passed - backend: " << model->get_model_type()
              << ", mode: " << mode
              << ", observations: " << observation_dim
              << ", history: " << history_length
              << ", actions: " << actions.size() << std::endl;
    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        return RunValidation(argc, argv);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Policy model validation failed: " << error.what() << std::endl;
        return 1;
    }
}

#include "rl_sdk.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

class OfflinePolicyValidator : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>* state) override
    {
        (void)state;
    }

    void SetCommand(const RobotCommand<float>* command) override
    {
        (void)command;
    }
};

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <robot_name> <policy_name>" << std::endl;
        return 2;
    }

    try
    {
        const std::string robot_name = argv[1];
        const std::string policy_name = argv[2];
        OfflinePolicyValidator validator;
        validator.robot_name = robot_name;
        validator.ReadYaml(robot_name, "base.yaml");
        validator.InitRL(robot_name + "/" + policy_name);
        std::cout << "Policy configuration validation passed: "
                  << robot_name << "/" << policy_name << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Policy configuration validation failed: " << error.what() << std::endl;
        return 1;
    }
}

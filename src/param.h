#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <rclcpp/utilities.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    std::string command_topic = "/mujoco_sim/command";
    std::string state_topic = "/mujoco_sim/state";
    double kp = 20.0;
    double kd = 1.0;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            if (cfg["command_topic"]) command_topic = cfg["command_topic"].as<std::string>();
            if (cfg["state_topic"]) state_topic = cfg["state_topic"].as<std::string>();
            if (cfg["kp"]) kp = cfg["kp"].as<double>();
            if (cfg["kd"]) kd = cfg["kd"].as<double>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("MuJoCo ROS 2 simulator");
    desc.add_options()
        ("help,h", "Show help message")
        ("command-topic", po::value<std::string>(&config.command_topic), "ROS2 command topic")
        ("state-topic", po::value<std::string>(&config.state_topic), "ROS2 state topic")
        ("robot,r", po::value<std::string>(&config.robot), "Model label; -r pendulum")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "MuJoCo XML or MJB model path")
    ;

    // ros2 launch appends --ros-args (for example, __node remappings). Those
    // arguments belong to rclcpp and must not be parsed as simulator options.
    const auto ros_filtered = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<const char*> simulator_argv;
    simulator_argv.reserve(ros_filtered.size());
    for (const auto& argument : ros_filtered) {
        simulator_argv.push_back(argument.c_str());
    }

    po::variables_map vm;
    po::store(po::parse_command_line(
                  static_cast<int>(simulator_argv.size()), simulator_argv.data(), desc),
              vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}

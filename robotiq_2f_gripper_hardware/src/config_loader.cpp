#include <robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace robotiq_2f_gripper_hardware
{

    void GripperNode::loadConfig(const std::string &config_file)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(config_file);

            // Load hardware settings with defaults
            if (config["hardware"])
            {
                serial_port_ = config["hardware"]["serial_port"].as<std::string>("/dev/ttyUSB0");
                baudrate_ = config["hardware"]["baudrate"].as<int>(115200);
                timeout_ = config["hardware"]["timeout"].as<double>(1.0);
                action_timeout_ = config["hardware"]["action_timeout"].as<int>(20);
                slave_address_ = config["hardware"]["slave_address"].as<int>(9);
                fake_hardware_ = config["hardware"]["fake_hardware"].as<bool>(false);

                RCLCPP_INFO(get_logger(), "Hardware settings loaded from config file");
            }
            else
            {
                // Set defaults if hardware section is missing
                serial_port_ = "/dev/ttyUSB0";
                baudrate_ = 115200;
                timeout_ = 1.0;
                action_timeout_ = 20;
                slave_address_ = 9;
                fake_hardware_ = false;

                RCLCPP_WARN(get_logger(), "Hardware section missing in config file, using defaults");
            }

            // Load topic names
            if (config["topics"])
            {
                YAML::Node topics = config["topics"];
                for (const auto &it : topics)
                {
                    std::string key = it.first.as<std::string>();
                    std::string value = it.second.as<std::string>();
                    topic_names_[key] = value;
                    RCLCPP_INFO(get_logger(), "Loaded topic name: %s = %s", key.c_str(), value.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Topics section missing in config file");
            }

            // Load constants with defaults
            if (config["constants"])
            {
                OPEN_THRESHOLD = config["constants"]["open_threshold"].as<double>(0.2);
                CLOSE_THRESHOLD = config["constants"]["close_threshold"].as<double>(-0.2);

                RCLCPP_INFO(get_logger(), "Constants loaded from config file");
            }
            else
            {
                // Set defaults if constants section is missing
                OPEN_THRESHOLD = 0.2;
                CLOSE_THRESHOLD = -0.2;

                RCLCPP_WARN(get_logger(), "Constants section missing in config file, using defaults");
            }

            RCLCPP_INFO(get_logger(), "Config file loaded successfully: %s", config_file.c_str());
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(get_logger(), "Error opening YAML file: %s - %s", config_file.c_str(), e.what());
            // Use defaults in case of error
            serial_port_ = "/dev/ttyUSB0";
            baudrate_ = 115200;
            timeout_ = 1.0;
            action_timeout_ = 20;
            slave_address_ = 9;
            fake_hardware_ = false;
            OPEN_THRESHOLD = 0.2;
            CLOSE_THRESHOLD = -0.2;
        }
    }

} // namespace robotiq_2f_gripper_hardware

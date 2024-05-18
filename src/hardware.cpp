#include "rclcpp/rclcpp.hpp"

#include <fmt/format.h>

#include "hardware.h"
#include "servo.h"

namespace zebra_zero
{
    int velocity_list[] = {
        100000,
        100000,
        100000,
        100000,
        200000,
        100000,
    };

    int acceleration_list[] = {
        10000,
        10000,
        10000,
        10000,
        20000,
        10000,
    };

    RobotSystem::~RobotSystem()
    {
        // on ctrl-c, on_deactivate isn't called, so do it here.
        // see: https://github.com/ros-controls/ros2_control/issues/472
        this->on_deactivate(rclcpp_lifecycle::State());
    }

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        nmc = new NmcBus("/dev/ttyUSB0", 19200);
        modules_ = 0;
        position_active_ = false;
        velocity_active_ = false;

        modules_ = nmc->init();
        if (modules_ == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ZebraZeroHardware"), "Found no NMC modules. Check that Robot is powered on.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Found: %d modules", modules_);

        for (int i = 0; i < modules_; i++)
        {
            Servo *servo = (Servo *)nmc->module(i);
            if (!servo->zero())
            {
                RCLCPP_ERROR(rclcpp::get_logger("ZebraZeroHardware"), "Failed to zero module %d", i);
            }
        }

        home_position_.assign(6, 0);
        home_position_[1] = M_PI / 2;
        home_position_[2] = -M_PI / 2;
        home_position_[4] = -M_PI / 2;

        joint_position_.assign(6, 0);
        joint_position_command_ = home_position_;
        joint_velocity_.assign(6, 0);
        joint_velocity_command_.assign(6, 0);

        for (const auto &joint : info_.joints)
        {
            for (const auto &interface : joint.state_interfaces)
            {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        if (hardware_interface::SystemInterface::on_activate(previous_state) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        if (hardware_interface::SystemInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Deactivating!");

        delete (nmc);
        nmc = NULL;
        modules_ = 0;
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }
        ind = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocity_[ind++]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
        }
        ind = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocity_command_[ind++]);
        }

        return command_interfaces;
    }

    return_type RobotSystem::read(__attribute__((unused)) const rclcpp::Time &time, __attribute__((unused)) const rclcpp::Duration &period)
    {
        // TODO: move this into the constructor
        std::vector<int> encoders;
        std::vector<int> encoder_velocity;
        std::vector<double> zeros;
        encoders.assign(6, 0);
        encoder_velocity.assign(6, 0);
        zeros.assign(6, 0);

        for (int i = 0; i < modules_; i++)
        {
            int pos, vel;
            Servo *servo = (Servo *)nmc->module(i);
            servo->read(&pos, &vel);
            encoders[i] = pos;
            encoder_velocity[i] = vel;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ZebraZeroHardware"), "encoders: %d %d %d", encoders[3], encoders[4], encoders[5]);

        RobotSystem::encoders_to_angles(encoders, this->joint_position_, this->home_position_);
        RobotSystem::encoders_to_angles(encoder_velocity, this->joint_velocity_, zeros);

        return return_type::OK;
    }

    return_type RobotSystem::write(__attribute__((unused)) const rclcpp::Time &time, __attribute__((unused)) const rclcpp::Duration &period)
    {
        if (this->position_active_)
        {
            move_direct();
            // move_path(126);
        }
        if (this->velocity_active_)
        {
            move_velocity();
        }

        return return_type::OK;
    }

    hardware_interface::CallbackReturn RobotSystem::on_shutdown(__attribute__((unused)) const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Shutting down Zebra Zero");

        delete (this->nmc);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void RobotSystem::encoders_to_angles(const std::vector<int> &encoders, std::vector<double> &angles, const std::vector<double> &home)
    {
        for (int i = 0; i < 3; i++)
        {
            angles[i] = (encoders[i] / 30557.75) + home[i];
        }
        double ec3 = encoders[3]; // * 17.33 / 24.0;
        double ec4 = encoders[4]; // * 17.33 / 24.0;
        double ec5 = encoders[5]; // * 17.33 / 24.0;

        angles[3] = ec3 / 11034.53 - encoders[2] / 20371.83 + home[3];
        angles[4] = ec5 / 22069.06 + ec4 / 44138.12 + encoders[2] / 27162.44 + home[4];
        angles[5] = ec5 / 11034.53 - ec4 / 22069.06 + ec3 / 11034.53 - encoders[2] / 40743.68 + home[5];
    }

    void RobotSystem::angles_to_encoders(const std::vector<double> &angles, std::vector<int> &encoders, const std::vector<double> &home)
    {
        for (int i = 0; i < 3; i++)
        {
            encoders[i] = (angles[i] - home[i]) * 30557.75;
        }

        double temp3 = 16551.79 * (angles[2] - home[2]);
        double temp4 = 11034.53 * (angles[3] - home[3]);

        encoders[3] = temp3 + temp4;
        encoders[4] = -temp3 + temp4 + 22069.06 * (angles[4] - home[4]) - 11034.53 * (angles[5] - home[5]);
        encoders[5] = -temp3 + 11034.53 * (angles[4] - home[4]) + 5517.265 * (home[3] - angles[3] + angles[5] - home[5]);
    }

    void RobotSystem::move_velocity()
    {
        std::vector<int> encoder;
        encoder.assign(6, 0);

        std::vector<double> zeros;
        zeros.assign(6, 0);

        RobotSystem::angles_to_encoders(this->joint_velocity_command_, encoder, zeros);
        RCLCPP_DEBUG(rclcpp::get_logger("ZebraZeroHardware"),
                     "Setting velocity to %d %d %d %d %d %d", encoder[0], encoder[1], encoder[2], encoder[3], encoder[4], encoder[5]);

        for (int i = 0; i < modules_; i++)
        {
            ((Servo *)nmc->module(i))->velocity(encoder[i], encoder[i]);
        }
    }

    void RobotSystem::move_direct()
    {
        std::vector<int> encoder;
        encoder.assign(6, 0);

        RobotSystem::angles_to_encoders(this->joint_position_command_, encoder, home_position_);

        for (int i = 0; i < modules_; i++)
        {
            ((Servo *)nmc->module(i))->write(encoder[i], velocity_list[i], acceleration_list[i]);
        }
    }

    void RobotSystem::move_path(int point_count)
    {
        if (nmc->moving())
        {
            RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Already moving.");
            return;
        }
        std::vector<int> encoder;
        encoder.assign(6, 0);

        std::vector<double> point;
        point.assign(6, 0);

        std::vector<long> paths[6];

        nmc->initPath();
        for (int joint = 0; joint < modules_; joint++)
        {
            paths[joint].assign(point_count, 0);
        }

        for (int p = 0; p < point_count; p++)
        {
            for (int joint = 0; joint < modules_; joint++)
            {
                double delta = (joint_position_command_[joint] - joint_position_[joint]);
                point[joint] = joint_position_[joint] + (delta * (p + 1)) / point_count;
            }
            RobotSystem::angles_to_encoders(point, encoder, home_position_);
            for (int joint = 0; joint < modules_; joint++)
            {
                paths[joint][p] = encoder[joint];
            }
        }

        for (int joint = 0; joint < modules_; joint++)
        {
            ((Servo *)nmc->module(joint))->setPath(paths[joint]);
        }
        nmc->startPath();
    }

    hardware_interface::return_type RobotSystem::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
    {
        // Taken from https://github.com/frankaemika/franka_ros2/blob/humble/franka_hardware/src/franka_hardware_interface.cpp
        auto contains_interface_type = [](const std::string &interface,
                                          const std::string &interface_type)
        {
            size_t slash_position = interface.find('/');
            if (slash_position != std::string::npos && slash_position + 1 < interface.size())
            {
                std::string after_slash = interface.substr(slash_position + 1);
                return after_slash == interface_type;
            }
            return false;
        };

        auto generate_error_message = [this](const std::string &start_stop_command,
                                             const std::string &interface_name,
                                             size_t actual_interface_size,
                                             size_t expected_interface_size)
        {
            std::string error_message =
                fmt::format("Invalid number of {} interfaces to {}. Expected {}, given {}", interface_name,
                            start_stop_command, expected_interface_size, actual_interface_size);
            RCLCPP_FATAL(rclcpp::get_logger("ZebraZeroHardware"), "%s", error_message.c_str());

            throw std::invalid_argument(error_message);
        };

        for (const auto &interface_type : {"position", "velocity"})
        {
            size_t num_stop_interface =
                std::count_if(stop_interfaces.begin(), stop_interfaces.end(),
                              [contains_interface_type, &interface_type](const std::string &interface_given)
                              {
                                  return contains_interface_type(interface_given, interface_type);
                              });
            size_t num_start_interface =
                std::count_if(start_interfaces.begin(), start_interfaces.end(),
                              [contains_interface_type, &interface_type](const std::string &interface_given)
                              {
                                  return contains_interface_type(interface_given, interface_type);
                              });

            if (num_stop_interface == 6)
            {
                stop_mode_ = interface_type;
            }
            else if (num_stop_interface != 0U)
            {
                generate_error_message("stop", interface_type, num_stop_interface, 6);
            }
            if (num_start_interface == 6)
            {
                start_mode_ = interface_type;
            }
            else if (num_start_interface != 0U)
            {
                generate_error_message("start", interface_type, num_start_interface, 6);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotSystem::perform_command_mode_switch(
        __attribute__((unused)) const std::vector<std::string> &start_interfaces,
        __attribute__((unused)) const std::vector<std::string> &stop_interfaces)
    {
        if (start_mode_ == "position")
        {
            position_active_ = true;
            velocity_active_ = false;
            RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Position mode activated.");
        }
        else if (start_mode_ == "velocity")
        {
            position_active_ = false;
            velocity_active_ = true;
            RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Velocity mode activated.");
        }
        if (stop_mode_ == "position")
        {
            position_active_ = false;
            // TODO: actually stop robot here.
            RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Position mode deactivated.");
        }
        else if (stop_mode_ == "velocity")
        {
            velocity_active_ = false;
            // TODO: actually stop robot here.
            RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Velocity mode deactivated.");
        }

        // clear out mode switches
        start_mode_ = "";
        stop_mode_ = "";

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zebra_zero::RobotSystem, hardware_interface::SystemInterface)

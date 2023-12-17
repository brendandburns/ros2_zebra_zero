#include "hardware.h"

namespace zebra_zero
{
    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        // setup communication with robot hardware
        // ...

        joint_position_.assign(6, 0);
        joint_position_command_.assign(6, 0);
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

    return_type RobotSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // read hardware values for state interfaces, e.g joint encoders and sensor readings
        // ...
        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // send command interface values to hardware, e.g joint set joint velocity
        // ...
        return return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zebra_zero::RobotSystem, hardware_interface::SystemInterface)
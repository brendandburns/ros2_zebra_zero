#include "rclcpp/rclcpp.hpp"

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

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        nmc = new NmcBus("/dev/ttyUSB0", 19200);
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

    return_type RobotSystem::read(__attribute__ ((unused)) const rclcpp::Time &time, __attribute__ ((unused)) const rclcpp::Duration &period)
    {
        // TODO: move this into the constructor
        std::vector<int> encoders;
        encoders.assign(6, 0);

        for (int i = 0; i < modules_; i++)
        {
            Servo *servo = (Servo *)nmc->module(i);
            encoders[i] = servo->read();
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ZebraZeroHardware"), "encoders: %d %d %d", encoders[3], encoders[4], encoders[5]);

        this->encoders_to_angles(encoders, this->joint_position_);

        return return_type::OK;
    }

    return_type RobotSystem::write(__attribute__ ((unused)) const rclcpp::Time &time, __attribute__ ((unused)) const rclcpp::Duration &period)
    {

        return return_type::OK;
    }

    hardware_interface::CallbackReturn RobotSystem::on_shutdown(__attribute__ ((unused)) const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ZebraZeroHardware"), "Shutting down Zebra Zero");

        delete (this->nmc);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void RobotSystem::encoders_to_angles(const std::vector<int> &encoders, std::vector<double> &angles)
    {
        for (int i = 0; i < 3; i++)
        {
            angles[i] = (encoders[i] / 30557.75) + this->home_position_[i];
        }
        double ec3 = encoders[3]; // * 17.33 / 24.0;
        double ec4 = encoders[4]; // * 17.33 / 24.0;
        double ec5 = encoders[5]; // * 17.33 / 24.0;

        angles[3] = ec3 / 11034.53 - encoders[2] / 20371.83 + this->home_position_[3];
        angles[4] = ec5 / 22069.06 + ec4 / 44138.12 + encoders[2] / 27162.44 + this->home_position_[4];
        angles[5] = ec5 / 11034.53 - ec4 / 22069.06 + ec3 / 11034.53 - encoders[2] / 40743.68 + this->home_position_[5];
    }

    void RobotSystem::angles_to_encoders(const std::vector<double> &angles, std::vector<int> &encoders)
    {
        for (int i = 0; i < 3; i++)
        {
            encoders[i] = (angles[i] - home_position_[i]) * 30557.75;
        }

        double temp3 = 16551.79 * (angles[2] - home_position_[2]);
        double temp4 = 11034.53 * (angles[3] - home_position_[3]);

        encoders[3] = temp3 + temp4;
        encoders[4] = -temp3 + temp4 + 22069.06 * (angles[4] - home_position_[4]) - 11034.53 * (angles[5] - home_position_[5]);
        encoders[5] = -temp3 + 11034.53 * (angles[4] - home_position_[4]) + 5517.265 * (home_position_[3] - angles[3] + angles[5] - home_position_[5]);
    }

    void RobotSystem::move_direct()
    {
        std::vector<int> encoder;
        encoder.assign(6, 0);

        this->angles_to_encoders(this->joint_position_command_, encoder);

        for (int i = 0; i < modules_; i++)
        {
            ((Servo *)nmc->module(i))->write(encoder[i], velocity_list[i], acceleration_list[i]);
        }
    }

    void RobotSystem::move_path(int point_count)
    {
        std::vector<int> encoder;
        encoder.assign(6, 0);

        std::vector<double> point;
        point.assign(6, 0);

        std::vector<long> paths[6];

        nmc->initPath();
        for (int joint = 0; joint < 6; joint++) {
            paths[joint].assign(point_count, 0);
        }

        for (int p = 1; p <= point_count; p++) {
            for (int joint = 0; joint < 6; joint++) {
                double delta = (joint_position_command_[joint] - joint_position_[joint]);
                point[joint] = joint_position_[joint] + (delta * p) / point_count;
            }
            this->angles_to_encoders(point, encoder);
            for (int joint = 0; joint < 6; joint++) {
                paths[joint][p] = encoder[joint];
            }
        }

        for (int joint = 0; joint < 6; joint++) {
            ((Servo *)nmc->module(joint))->setPath(paths[joint]);
        }
        nmc->startPath();
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zebra_zero::RobotSystem, hardware_interface::SystemInterface)
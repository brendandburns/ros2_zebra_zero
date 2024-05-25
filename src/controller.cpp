#include "controller.h"

using config_type = controller_interface::interface_configuration_type;

namespace zebra_zero
{
    RobotController::RobotController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn RobotController::on_init()
    {
        // TODO consider using this instead:
        // #include <rbdl/addons/urdfreader/urdfreader.h>

        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        point_interp_.positions.assign(joint_names_.size(), 0);
        point_interp_.velocities.assign(joint_names_.size(), 0);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration RobotController::command_interface_configuration()
        const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return conf;
    }

    controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return conf;
    }

    controller_interface::CallbackReturn RobotController::on_configure(const rclcpp_lifecycle::State &)
    {
/*
          auto callback =
            [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
          {
            traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
            new_msg_ = true;
          };

          joint_command_subscriber_ =
            get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
              "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);
*/
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_activate(const rclcpp_lifecycle::State &)
    {
          // clear out vectors in case of restart
          joint_effort_command_interface_.clear();
          joint_position_state_interface_.clear();
          
          // assign command interfaces
          for (auto & interface : command_interfaces_)
          {
            command_interface_map_[interface.get_interface_name()]->push_back(interface);
          }

          // assign state interfaces
          for (auto & interface : state_interfaces_)
          {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
          }
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type RobotController::update(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        auto shoulder = joint_position_state_interface_[1].get().get_value();
        auto elbow = joint_position_state_interface_[2].get().get_value();
        auto elbow_actual = shoulder + -elbow;

        // calculate gravity compensation torque given shoulder and elbow angles
        // TODO: extract these constants out into controller parameters
        auto elbow_torque = sin(elbow_actual) * 6;
        auto shoulder_torque = sin(shoulder) * -6.5 - elbow_torque;

        RCLCPP_DEBUG(get_node()->get_logger(), "Shoulder Torque: %f, Elbow Torque: %f", shoulder_torque, elbow_torque);

        // Waist joint is vertical, no gravity compensation needed.
        joint_effort_command_interface_[0].get().set_value(0);
        // Compensate for shoulder and elbow
        joint_effort_command_interface_[1].get().set_value(int(shoulder_torque));
        joint_effort_command_interface_[2].get().set_value(int(elbow_torque));
        // For now there's no hand, so no need to compensate in the wrist
        joint_effort_command_interface_[3].get().set_value(0);
        joint_effort_command_interface_[4].get().set_value(0);
        joint_effort_command_interface_[5].get().set_value(0);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn RobotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_error(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn RobotController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

} // namespace zebra_zero

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    zebra_zero::RobotController, controller_interface::ControllerInterface)
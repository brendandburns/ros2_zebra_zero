#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nmc.h"

using hardware_interface::return_type;

namespace zebra_zero
{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
    {
    private:
        std::vector<double> home_position_;
        std::vector<double> joint_position_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_velocity_command_;

        bool position_active_;
        bool velocity_active_;
        std::string stop_mode_;
        std::string start_mode_;

        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
            {"position", {}}, {"velocity", {}}};

        NmcBus* nmc;
        int     modules_;

    public:
        ~RobotSystem();

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

         hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

    private:
        void encoders_to_angles(const std::vector<int> &encoders, std::vector<double> &angles);
        void angles_to_encoders(const std::vector<double> &angles, std::vector<int> &encoders, const std::vector<double> home);
        
        void move_direct();
        void move_path(int num_points);
        void set_velocity();

        // private members
        // ...
    };
}
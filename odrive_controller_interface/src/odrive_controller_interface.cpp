#include <memory>
#include <vector>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "odrive_types/srv/clear_error.hpp"
#include "odrive_types/srv/set_absolute_position.hpp"

#include "odrive_controller_interface/odrive_controller_interface_parameters.hpp"


namespace odrive_controller_interface {

using trigger_command = double;
using uint32_state = double;
using uint8_state = double;

using BoolMsg = std_msgs::msg::Bool;

using TriggerSrv = std_srvs::srv::Trigger;
using ClearErrorSrv = odrive_types::srv::ClearError;
using SetAbsolutePositionSrv = odrive_types::srv::SetAbsolutePosition;

using CallbackReturn = controller_interface::CallbackReturn;
using MapOfReferencesToCommandInterfaces = std::unordered_map<
  std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;

class BetterODriveControllerInterface : public controller_interface::ControllerInterface {

public:
  BetterODriveControllerInterface();

  
  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
      
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
          
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
        
    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
        
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
protected:
    MapOfReferencesToCommandInterfaces command_interfaces_map_ = {};

    // Parameters from ROS for diff_drive_controller
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // Subscribers
    rclcpp::Subscription<BoolMsg>::SharedPtr estop_sub_{};

    // Clients
    rclcpp::Service<TriggerSrv>::SharedPtr clear_all_errors_srv_{};
    rclcpp::Service<ClearErrorSrv>::SharedPtr clear_errors_srv_{};
    rclcpp::Service<SetAbsolutePositionSrv>::SharedPtr set_absolute_position_srv_{};

    int estop_ = -1;
    bool clear_all_errors_ = false;
    std::vector<bool> clear_errors_cmd_ = {};
    std::vector<double> set_absolute_positions_cmd_ = {};

    // Joints
    std::vector<std::string> joint_names_ = {};
    std::vector<std::string> absolute_joint_names_ = {};
};


} // namespace odrive_controller_interface

using namespace odrive_controller_interface;

BetterODriveControllerInterface::BetterODriveControllerInterface() {}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_init()
{
    try {
        param_listener_ = std::make_shared<odrive_controller_interface::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown during init stage with message: " << e.what() << std::endl);
        return CallbackReturn::ERROR;
    }
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    try {
        // Reset vectors
        joint_names_ = {};
        absolute_joint_names_ = {};
        set_absolute_positions_cmd_ = {};
        clear_errors_cmd_ = {};
        
        // Register all joints
        for (auto joint_name : params_.joint_names) {
            joint_names_.emplace_back(joint_name);
        }
        clear_errors_cmd_.resize(joint_names_.size(), false);
        
        // Register absolute joints
        for (auto joint_name : params_.absolute_position_names) {
            absolute_joint_names_.emplace_back(joint_name);
        }
        set_absolute_positions_cmd_.resize(absolute_joint_names_.size(), NAN);

        
        estop_sub_ = get_node()->create_subscription<BoolMsg>(
            "~/estop", rclcpp::SystemDefaultsQoS(),
            [this](const BoolMsg::SharedPtr msg) { estop_ = msg->data ? 1 : 0; });
            
        clear_all_errors_srv_ = get_node()->create_service<TriggerSrv>(
            "~/clear_all_errors",
            [this](const TriggerSrv::Request::SharedPtr,
                TriggerSrv::Response::SharedPtr) {
                clear_all_errors_ = true;
            });
        
        clear_errors_srv_ = get_node()->create_service<ClearErrorSrv>(
            "~/clear_errors",
            [this](const ClearErrorSrv::Request::SharedPtr request,
                ClearErrorSrv::Response::SharedPtr response) {
                    for (auto joint_name : request->joint_names) {
                        auto match = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
                        if (match != joint_names_.end()) {
                            auto index = match - joint_names_.begin();
                            clear_errors_cmd_[index] = true;
                        } else {
                            RCLCPP_WARN_STREAM(get_node()->get_logger(), "No joint named '" << joint_name << "'");
                        }
                    }
                });
        
        set_absolute_position_srv_ = get_node()->create_service<SetAbsolutePositionSrv>(
            "~/set_absolute_positions",
            [this](const SetAbsolutePositionSrv::Request::SharedPtr request,
                SetAbsolutePositionSrv::Response::SharedPtr response) {
                    if (request->joint_names.size() != request->positions.size()) {
                        RCLCPP_ERROR(get_node()->get_logger(), "Size mismatch in 'SetAbsolutePositionSrv'");
                        return;
                    }
                    for (auto joint_name : request->joint_names) {
                        auto match = std::find(absolute_joint_names_.begin(), absolute_joint_names_.end(), joint_name);
                        if (match != absolute_joint_names_.end()) {
                            auto index = match - absolute_joint_names_.begin();
                            set_absolute_positions_cmd_[index] = request->positions[index];
                        } else {
                            RCLCPP_WARN_STREAM(get_node()->get_logger(), "No joint named '" << joint_name << "'");
                        }
                    }
                });
        
        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown during configure stage with message: " << e.what());
        return CallbackReturn::ERROR;
    }
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    for (auto joint_name : joint_names_) {
        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "estop";
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/estop", std::ref(*match));
            }
        }

        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "clear_error_cmd";
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/clear_error_cmd", std::ref(*match));
            }
        }
    }

    for (auto joint_name : absolute_joint_names_) {
        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "set_absolute_position";
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/set_absolute_position", std::ref(*match));
            }
        }

        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "set_absolute_position_cmd";
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/set_absolute_position_cmd", std::ref(*match));
            }
        }
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    command_interfaces_map_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BetterODriveControllerInterface::command_interface_configuration() const
{
    auto config = controller_interface::InterfaceConfiguration();
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (auto joint_name : joint_names_) {
        config.names.emplace_back(joint_name + "/estop");
        config.names.emplace_back(joint_name + "/clear_errors_cmd");
    }

    for (auto joint_name : absolute_joint_names_) {
        config.names.emplace_back(joint_name + "/set_absolute_pos");
        config.names.emplace_back(joint_name + "/set_absolute_pos_cmd");
    }
    return config;
}

controller_interface::InterfaceConfiguration BetterODriveControllerInterface::state_interface_configuration() const
{
    auto config = controller_interface::InterfaceConfiguration();
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

controller_interface::return_type BetterODriveControllerInterface::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    try {
        for (int i = 0; i < joint_names_.size(); ++i) {
            // Clear error commands
            if (clear_all_errors_ || clear_errors_cmd_[i]) {
                command_interfaces_map_.at(joint_names_[i] + "/clear_error_cmd").get().set_value(1.0);
                clear_errors_cmd_[i] = false; // Consume command
            }
            // E-Stop command
            if (estop_ != -1) {
                command_interfaces_map_.at(joint_names_[i] + "/estop").get().set_value(estop_);
            }
        }
    
        for (int i = 0; i < absolute_joint_names_.size(); ++i) {
            // Set absolute position commands
            if (set_absolute_positions_cmd_[i] != NAN) {
                command_interfaces_map_.at(absolute_joint_names_[i] + "/set_absolute_position").get().set_value(set_absolute_positions_cmd_[i]);
                command_interfaces_map_.at(absolute_joint_names_[i] + "/set_absolute_position_cmd").get().set_value(1.0);
                set_absolute_positions_cmd_[i] = NAN; // Consume command
            }
        }
    } catch (std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Exception thrown during update stage with message: " << e.what());
        return controller_interface::return_type::ERROR;
    }

    // Consume commands
    clear_all_errors_ = false;
    estop_ = -1;
    return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(odrive_controller_interface::BetterODriveControllerInterface, controller_interface::ControllerInterface)
    
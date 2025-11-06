#include <memory>
#include <vector>
#include <chrono>
#include <mutex>
#include <type_traits>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "odrive_types/srv/clear_error.hpp"
#include "odrive_types/srv/set_absolute_position.hpp"
#include "odrive_types/msg/joint_bool.hpp"

#include "odrive_controller_interface/odrive_controller_interface_parameters.hpp"


namespace odrive_controller_interface {

using trigger_command = double;
using uint32_state = double;
using uint8_state = double;

using BoolMsg = std_msgs::msg::Bool;
using JointBoolMsg = odrive_types::msg::JointBool;

using TriggerSrv = std_srvs::srv::Trigger;
using ClearErrorSrv = odrive_types::srv::ClearError;
using SetAbsolutePositionSrv = odrive_types::srv::SetAbsolutePosition;

using CallbackReturn = controller_interface::CallbackReturn;
using MapOfReferencesToCommandInterfaces = std::unordered_map<
  std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;

template <typename T>
class AtomicEventArray {
public:

    AtomicEventArray() noexcept {}
    AtomicEventArray(size_t size) noexcept {
        reserve(size);
    }
    AtomicEventArray(size_t size, const T& v) noexcept {
        resize(size, v);
    }

    void reserve(size_t size) noexcept {
        if (_values) {
            delete[] _values;
        }
        if (_available) {
            delete[] _available;
        }
        _values = new std::atomic<T>[size];
        _available = new std::atomic<bool>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _available[i].store(false, std::memory_order_relaxed);
        }
    }

    void reserve(size_t size) volatile noexcept {
        if (_values) {
            delete[] _values;
        }
        if (_available) {
            delete[] _available;
        }
        _values = new std::atomic<T>[size];
        _available = new std::atomic<bool>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _available[i].store(false, std::memory_order_relaxed);
        }
    }

    void resize(size_t size, const T& v) noexcept {
        if (_values) {
            delete[] _values;
        }
        if (_available) {
            delete[] _available;
        }
        _values = new std::atomic<T>[size];
        _available = new std::atomic<bool>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _values[i].store(v, std::memory_order_relaxed);
            _available[i].store(false, std::memory_order_relaxed);
        }
    }

    void resize(size_t size, const T& v) volatile noexcept {
        if (_values) {
            delete[] _values;
        }
        if (_available) {
            delete[] _available;
        }
        _values = new std::atomic<T>[size];
        _available = new std::atomic<bool>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _values[i].store(v, std::memory_order_relaxed);
            _available[i].store(false, std::memory_order_relaxed);
        }
    }

    operator bool() const noexcept {
        for (auto i = _available; i < _available + _size; ++i) {
            if (i->load(std::memory_order_relaxed)) return true;
        }
        return false;
    }

    operator bool() const volatile noexcept {
        for (auto i = _available; i < _available + _size; ++i) {
            if (i->load(std::memory_order_relaxed)) return true;
        }
        return false;
    }

    bool write(size_t i, T v) noexcept {
        if (i >= _size) return false;
        _values[i].store(v, std::memory_order_relaxed);
        _available[i].store(true, std::memory_order_relaxed);
        return true;
    }

    bool write(size_t i, T v) volatile noexcept {
        if (i >= _size) return false;
        _values[i].store(v, std::memory_order_relaxed);
        _available[i].store(true, std::memory_order_relaxed);
        return true;
    }

    bool read(size_t i, T& v) noexcept {
        if (i >= _size) return false;
        if (_available[i].load(std::memory_order_relaxed)) {
            v = _values[i].load(std::memory_order_relaxed);
            _available[i].store(false, std::memory_order_relaxed);
            return true;
        }
        return false;
    }

    bool read(size_t i, T& v) volatile noexcept {
        if (i >= _size) return false;
        if (_available[i].load(std::memory_order_relaxed)) {
            v = _values[i].load(std::memory_order_relaxed);
            _available[i].store(false, std::memory_order_relaxed);
            return true;
        }
        return false;
    }

    bool read_without_notify(size_t i, T& v) const noexcept {
        if (i >= _size) return false;
        if (_available[i].load(std::memory_order_relaxed)) {
            v = _values[i].load(std::memory_order_relaxed);
            return true;
        }
        return false;
    }

    bool read_without_notify(size_t i, T& v) const volatile noexcept {
        if (i >= _size) return false;
        if (_available[i].load(std::memory_order_relaxed)) {
            v = _values[i].load(std::memory_order_relaxed);
            return true;
        }
        return false;
    }

    void clear_events() noexcept {
        for (auto i = _available; i < _available + _size; ++i) {
            i->store(false, std::memory_order_relaxed);
        }
    }

    void clear_events() volatile noexcept {
        for (auto i = _available; i < _available + _size; ++i) {
            i->store(false, std::memory_order_relaxed);
        }
    }

    ~AtomicEventArray() noexcept {
        if (_values) {
            delete[] _values;
            _values = nullptr;
        }
        if (_available) {
            delete[] _available;
            _available = nullptr;
        }
    }

private:
    size_t _size = 0;
    std::atomic<bool>* _available = nullptr;
    std::atomic<T>* _values = nullptr;
};

template <typename T>
class AtomicArray {
public:
    AtomicArray() noexcept {}
    AtomicArray(size_t size) noexcept {
        reserve(size);
    }
    AtomicArray(size_t size, T v) noexcept {
        resize(size, v);
    }

    void reserve(size_t size) noexcept {
        if (_values) {
            delete[] _values;
        }
        _values = new std::atomic<T>[size];
        _size = size;
    }

    void resize(size_t size, T v) noexcept {
        if (_values) {
            delete[] _values;
        }
        _values = new std::atomic<T>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _values[i].store(v, std::memory_order_relaxed);
        }
    }

    void resize(size_t size, T v) volatile noexcept {
        if (_values) {
            delete[] _values;
        }
        _values = new std::atomic<T>[size];
        _size = size;
        for (int i = 0; i < size; ++i) {
            _values[i].store(v, std::memory_order_relaxed);
        }
    }

    bool write(size_t i, T v) noexcept {
        if (i >= _size) return false;
        _values[i].store(v, std::memory_order_relaxed);
        return true;
    }

    bool write(size_t i, T v) volatile noexcept {
        if (i >= _size) return false;
        _values[i].store(v, std::memory_order_relaxed);
        return true;
    }

    bool read(size_t i, T& v) const noexcept {
        if (i >= _size) return false;
        v = _values[i].load(std::memory_order_relaxed);
        return true;
    }

    bool read(size_t i, T& v) const volatile noexcept {
        if (i >= _size) return false;
        v = _values[i].load(std::memory_order_relaxed);
        return true;
    }

    ~AtomicArray() noexcept {
        if (_values) {
            delete[] _values;
            _values = nullptr;
        }
    }

private:
    size_t _size = 0;
    std::atomic<T>* _values = nullptr;
};

constexpr const char ODRIVE_IF_ENABLE[] = "enable";
constexpr const char ODRIVE_IF_CLEAR_ERRORS_CMD[] = "clear_errors_cmd";
constexpr const char ODRIVE_IF_SET_ABSOLUTE_POSITION[] = "set_absolute_position";

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
    std::vector<rclcpp::Subscription<BoolMsg>::SharedPtr> enable_subs_ = {};

    // Services
    rclcpp::Service<TriggerSrv>::SharedPtr clear_all_errors_srv_{};
    rclcpp::Service<ClearErrorSrv>::SharedPtr clear_errors_srv_{};
    rclcpp::Service<SetAbsolutePositionSrv>::SharedPtr set_absolute_position_srv_{};

    // Joints
    std::vector<std::string> joint_names_ = {};
    std::vector<std::string> absolute_joint_names_ = {};

    // Command values
    volatile AtomicArray<bool> joints_enable_ = {};
    volatile AtomicArray<bool> joints_clear_error_ = {};
    volatile AtomicEventArray<double> joints_set_absolute_position_ = {};
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
        
        // Register all joints
        for (auto joint_name : params_.joint_names) {
            joint_names_.emplace_back(joint_name);
        }
        joints_clear_error_.resize(joint_names_.size(), false);
        enable_subs_.reserve(joint_names_.size());
        joints_enable_.resize(joint_names_.size(), false);
        
        // Register absolute joints
        for (auto joint_name : params_.absolute_position_names) {
            absolute_joint_names_.emplace_back(joint_name);
        }
        joints_set_absolute_position_.resize(absolute_joint_names_.size(), 0);

        
        // estop_sub_ = get_node()->create_subscription<BoolMsg>(
        //     "~/estop", rclcpp::SystemDefaultsQoS(), 
        //     [this](const BoolMsg::SharedPtr msg) {
        //         rt_estop_.set(msg->data);
        //     });

        for (int i = 0; i < joint_names_.size(); ++i) {
            joints_enable_.write(i, false);
            enable_subs_.emplace_back(get_node()->create_subscription<BoolMsg>(
                "~/enable/" + joint_names_[i], rclcpp::SystemDefaultsQoS(),
                [this, i](const BoolMsg::SharedPtr msg) {
                    if (msg) {
                        joints_enable_.write(i, msg->data);
                    }
                }
            ));
        }

            
        clear_all_errors_srv_ = get_node()->create_service<TriggerSrv>(
            "~/clear_all_errors",
            [this](const TriggerSrv::Request::SharedPtr,
                TriggerSrv::Response::SharedPtr response) {
                    response->message = "";
                    response->success = true;
                    for (int i = 0; i < joint_names_.size(); ++i) {
                        if (!joints_clear_error_.write(i, true)) {
                            response->success = false;
                            response->message += "Index out of range\n";
                            return;
                        }
                    }
                });
        
        clear_errors_srv_ = get_node()->create_service<ClearErrorSrv>(
            "~/clear_errors",
            [this](const ClearErrorSrv::Request::SharedPtr request,
                ClearErrorSrv::Response::SharedPtr response) {
                    response->message = "";
                    response->success = true;
                    for (auto joint_name : request->joint_names) {
                        auto match = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
                        if (match != joint_names_.end()) {
                            auto index = match - joint_names_.begin();
                            if (!joints_clear_error_.write(index, true)) {
                                response->success = false;
                                response->message += "Index out of range\n";
                                return;
                            }
                        } else {
                            response->success = false;
                            response->message += "No axis named: " + joint_name + "\n";
                        }
                    }
                });
        
        set_absolute_position_srv_ = get_node()->create_service<SetAbsolutePositionSrv>(
            "~/set_absolute_positions",
            [this](const SetAbsolutePositionSrv::Request::SharedPtr request,
                SetAbsolutePositionSrv::Response::SharedPtr response) {
                    response->message = "";
                    response->success = true;
                    if (request->joint_names.size() != request->positions.size()) {
                        response->success = false;
                        response->message += "Size mismatch in 'SetAbsolutePositionSrv'\n";
                        return;
                    }
                    for (auto joint_name : request->joint_names) {
                        auto match = std::find(absolute_joint_names_.begin(), absolute_joint_names_.end(), joint_name);
                        if (match != absolute_joint_names_.end()) {
                            auto index = match - absolute_joint_names_.begin();
                            if (!joints_set_absolute_position_.write(index, request->positions[index])) {
                                response->success = false;
                                response->message += "Index out of range\n";
                            }
                        } else {
                            response->success = false;
                            response->message += "No axis marked as absolute named: " + joint_name + "\n"; 
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
    // rt_estop_.set(estop_);
    for (auto joint_name : joint_names_) {
        {
            // {
            //     auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
            //         [&](hardware_interface::LoanedCommandInterface& interface) {
            //             return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "estop";
            //         }
            //     );
            //     if (match != command_interfaces_.end()) {
            //         command_interfaces_map_.emplace(joint_name + "/estop", std::ref(*match));
            //     }
            // }
            {
                auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                    [&](hardware_interface::LoanedCommandInterface& interface) {
                        return interface.get_prefix_name() == joint_name && interface.get_interface_name() == ODRIVE_IF_ENABLE;
                    }
                );
                if (match != command_interfaces_.end()) {
                    command_interfaces_map_.emplace(joint_name + "/" + ODRIVE_IF_ENABLE, std::ref(*match));
                }
            }
        }

        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == ODRIVE_IF_CLEAR_ERRORS_CMD;
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/" + ODRIVE_IF_CLEAR_ERRORS_CMD, std::ref(*match));
            }
        }
    }

    for (auto joint_name : absolute_joint_names_) {
        {
            auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](hardware_interface::LoanedCommandInterface& interface) {
                    return interface.get_prefix_name() == joint_name && interface.get_interface_name() == ODRIVE_IF_SET_ABSOLUTE_POSITION;
                }
            );
            if (match != command_interfaces_.end()) {
                command_interfaces_map_.emplace(joint_name + "/" + ODRIVE_IF_SET_ABSOLUTE_POSITION, std::ref(*match));
            }
        }

        // {
        //     auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
        //         [&](hardware_interface::LoanedCommandInterface& interface) {
        //             return interface.get_prefix_name() == joint_name && interface.get_interface_name() == "set_absolute_position_cmd";
        //         }
        //     );
        //     if (match != command_interfaces_.end()) {
        //         command_interfaces_map_.emplace(joint_name + "/set_absolute_position_cmd", std::ref(*match));
        //     }
        // }
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    // rt_estop_.set(estop_);
    command_interfaces_map_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BetterODriveControllerInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    // if (joints_enable_) {
    //     delete[] joints_enable_;
    //     joints_enable_ = nullptr;
    // }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BetterODriveControllerInterface::command_interface_configuration() const
{
    auto config = controller_interface::InterfaceConfiguration();
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (auto joint_name : joint_names_) {
        // config.names.emplace_back(joint_name + "/estop");
        config.names.emplace_back(joint_name + "/" + ODRIVE_IF_ENABLE);
        config.names.emplace_back(joint_name + "/" + ODRIVE_IF_CLEAR_ERRORS_CMD);
    }

    for (auto joint_name : absolute_joint_names_) {
        config.names.emplace_back(joint_name + "/" + ODRIVE_IF_SET_ABSOLUTE_POSITION);
        // config.names.emplace_back(joint_name + "/set_absolute_pos_cmd");
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
            bool clear_error = false;
            if (joints_clear_error_.read(i, clear_error)) {
                command_interfaces_map_.at(joint_names_[i] + "/" + ODRIVE_IF_CLEAR_ERRORS_CMD).get().set_value(clear_error ? 1.0 : 0.0);
                joints_clear_error_.write(i, false); // Consume command
            } else {
                command_interfaces_map_.at(joint_names_[i] + "/" + ODRIVE_IF_CLEAR_ERRORS_CMD).get().set_value(0.0);
            }
            // E-Stop command
            // if (rt_estop_.get(estop_); estop_ != -1) {
            //     command_interfaces_map_.at(joint_names_[i] + "/estop").get().set_value(estop_);
            // }
            // bool enable = false;
            // if (rt_enable_.read(i, enable)) {
            //     command_interfaces_map_.at(joint_names_[i] + "/enable").get().set_value(enable);
            //     RCLCPP_INFO(get_node()->get_logger(), "Enabled");
            // } 
            // else {
            //     command_interfaces_map_.at(joint_names_[i] + "/enable").get().set_value(-1);
            // }
            bool enable = false;
            if (joints_enable_.read(i, enable)) {
                command_interfaces_map_.at(joint_names_[i] + "/" + ODRIVE_IF_ENABLE).get().set_value(enable ? 1.0 : 0.0);
            }
        }
    
        for (int i = 0; i < absolute_joint_names_.size(); ++i) {
            // Set absolute position commands
            double absolute_position = 0.0;
            if (joints_set_absolute_position_.read(i, absolute_position)) {
                command_interfaces_map_.at(absolute_joint_names_[i] + "/" + ODRIVE_IF_SET_ABSOLUTE_POSITION).get().set_value(absolute_position);
            } else {
                command_interfaces_map_.at(absolute_joint_names_[i] + "/" + ODRIVE_IF_SET_ABSOLUTE_POSITION).get().set_value(NAN);
            }
        }
    } catch (std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Exception thrown during update stage with message: " << e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(odrive_controller_interface::BetterODriveControllerInterface, controller_interface::ControllerInterface)
    
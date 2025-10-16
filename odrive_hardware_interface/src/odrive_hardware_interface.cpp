
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_hardware_interface {

class Axis;

using trigger_command = double;
using uint32_state = double;
using uint8_state = double;

class BetterODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
    double estop_timeout_ = 1.0;
    double idle_timeout_ = 2.0;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0; // [rad]
    double vel_setpoint_ = 0.0; // [rad/s]
    double torque_setpoint_ = 0.0; // [Nm]
    double estop_ = -1.0;
    double enable_ = -1.0;
    trigger_command clear_errors_cmd_ = 0.0;
    double set_absolute_pos_ = 0.0; // [rad]
    trigger_command set_absolute_pos_cmd_ = 0.0;

    // State (ODrives => ros2_control)
    uint32_state axis_error_ = 0;
    uint8_state axis_state_ = 0;
    uint8_state procedure_result_ = 0;
    uint8_state trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    double iq_setpoint_ = NAN;
    double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    uint32_state active_errors_ = 0;
    uint32_state disarm_reason_ = 0;
    double fet_temperature_ = NAN; // [C]
    double motor_temperature_ = NAN; // [C]
    double bus_voltage_ = NAN; // [V]
    double bus_current_ = NAN; // [A]
    double electrical_power_ = NAN; // [W]
    double mechanical_power_ = NAN; // [W]

    uint8_state protocol_version_ = 0;
    uint8_state hw_version_major_ = 0;
    uint8_state hw_version_minor_ = 0;
    uint8_state hw_version_variant_ = 0;
    uint8_state fw_version_major_ = 0;
    uint8_state fw_version_minor_ = 0;
    uint8_state fw_version_revision_ = 0;
    uint8_state fw_version_unreleased_ = 0;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    /**
     * @brief Set the axis state
     * 
     * @param state Requested axis state
     */
    void send_axis_state(const ODriveAxisState& state);
    /**
     * @brief Set the controller mode 
     * 
     * @param control Requested controller mode
     * @param input Requested input mode
     */
    void send_controller_mode(const ODriveControlMode& control, const ODriveInputMode& input);
    /**
     * @brief Clear errors
     * 
     * @param identify Set to 1 to flash the status led
     */
    void send_clear_errors(const uint8_t& identify = 0);
    /**
     * @brief Set the input position
     * 
     * @param input_pos Setpoint [rad]
     * @param velocity_feed_forward Feed forward velocity [rad/s]. 0 to use configured max
     * @param torque_feed_forward Feed forward torque [Nm]. 0 to use configured max
     */
    void send_input_pos(const double& input_pos, const double& velocity_feed_forward = 0, const double& torque_feed_forward = 0);
    /**
     * @brief Set the input velocity
     * 
     * @param velocity Setpoint [rad/s]
     * @param torque_feed_forward Feed forward torque [Nm]. 0 to use configured max
     */
    void send_input_vel(const double& velocity, const double& torque_feed_forward = 0);
    /**
     * @brief Set the input torque
     * 
     * @param torque Setpoint [Nm]
     */
    void send_input_torque(const double& torque);
    /**
     * @brief Set the E-Stop state
     * 
     * @param estop E-Stop state
     */
    void send_estop_state(const bool& estop);
    /**
     * @brief Set the absolute position
     * 
     * @param position Actual position [rad]
     */
    void send_absolute_position(const double& position);
    /**
     * @brief Set the velocity and current limits
     * 
     * @param velocity_limit Velocity limit [rad/s]
     * @param current_limit Current limit [A]
     */
    void send_limits(const double& velocity_limit, const double& current_limit);
    /**
     * @brief Set the trajectory velocity limit
     * 
     * @param limit Velocity limit [rad/s]
     */
    void send_trajectory_vel_limit(const double& limit);
    /**
     * @brief Set the trajectory accel/decel limits
     * 
     * @param accel_limit Acceleration limit [rad/s^2]
     * @param decel_limit Deceleration limit [rad/s^2]
     */
    void send_trajectory_accel_limits(const double& accel_limit, const double& decel_limit);
    /**
     * @brief Set the trajectory inertia
     * 
     * @param inertia Inertia [Nm/(rad/s^2)]
     */
    void send_trajectory_inertia(const double& inertia);

    template <typename T>
    void send(const T& msg) {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_hardware_interface

using namespace odrive_hardware_interface;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn BetterODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn BetterODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&BetterODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(rclcpp::get_logger("BetterODriveHardwareInterface"), "Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn BetterODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn BetterODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().
    return CallbackReturn::SUCCESS;
}

CallbackReturn BetterODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "deactivating ODrives...");

    for (auto& axis : axes_) {
        axis.send_axis_state(ODriveAxisState::AXIS_STATE_IDLE);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BetterODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "bus_voltage",
            &axes_[i].bus_voltage_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "bus_current",
            &axes_[i].bus_current_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "active_errors",
            &axes_[i].active_errors_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "disarm_reason",
            &axes_[i].disarm_reason_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "axis_state",
            &axes_[i].axis_state_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "axis_error",
            &axes_[i].axis_error_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "electrical_power",
            &axes_[i].electrical_power_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "fet_temperature",
            &axes_[i].fet_temperature_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "iq_measured",
            &axes_[i].iq_measured_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "iq_setpoint",
            &axes_[i].iq_setpoint_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "mechanical_power",
            &axes_[i].mechanical_power_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "motor_temperature",
            &axes_[i].motor_temperature_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "procedure_result",
            &axes_[i].procedure_result_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "torque_target",
            &axes_[i].torque_target_
        );
        state_interfaces.emplace_back(
            info_.joints[i].name,
            "trajectory_done_flag",
            &axes_[i].trajectory_done_flag_
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BetterODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
        command_interfaces.emplace_back(
            info_.joints[i].name,
            "set_absolute_position",
            &axes_[i].set_absolute_pos_
        );
        command_interfaces.emplace_back(
            info_.joints[i].name,
            "clear_errors_cmd",
            &axes_[i].clear_errors_cmd_
        );
        // command_interfaces.emplace_back(
        //     info_.joints[i].name,
        //     "estop",
        //     &axes_[i].estop_
        // );
        command_interfaces.emplace_back(
            info_.joints[i].name,
            "enable",
            &axes_[i].enable_
        );
    }

    return command_interfaces;
}

return_type BetterODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            ODriveControlMode control;
            ODriveInputMode input;
            if (axis.pos_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "Setting %s to position control", info_.joints[i].name.c_str());
                control = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
                input = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
            } else if (axis.vel_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "Setting %s to velocity control", info_.joints[i].name.c_str());
                control = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;
                input = ODriveInputMode::INPUT_MODE_VEL_RAMP;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("BetterODriveHardwareInterface"), "Setting %s to torque control", info_.joints[i].name.c_str());
                control = ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL;
                input = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
            }

            bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

            if (any_enabled) {
                axis.send_controller_mode(control, input); // Set control mode
            }
        }
    }

    return return_type::OK;
}

return_type BetterODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type BetterODriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration&) {
    static auto clk = rclcpp::Clock();
    for (auto& axis : axes_) {
        // Set absolute position
        if (!std::isnan(axis.set_absolute_pos_)) {
            RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("BetterODriveHardwareInterface"), clk, 1000, 
                "Seting absolute position for axis '" << axis.node_id_ << "' to '" << axis.set_absolute_pos_ << "'");
            axis.send_absolute_position(axis.set_absolute_pos_);
        }

        // // E-Stop
        // if (axis.estop_ < 0.0 && (axis.last_estop_timestamp_.seconds() + estop_timeout_) > time.seconds()) {
        //     // Timed out call estop
        //     axis.send_estop_state(true);
        //     RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("BetterODriveHardwareInterface"), clk, 1000, 
        //         "EStop timed out for axis '" << axis.node_id_ << "'");
        //     return return_type::OK;
        // } else if (axis.estop_ > 0.0) {
        //     RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("BetterODriveHardwareInterface"), clk, 1000, 
        //         "EStop called for axis '" << axis.node_id_ << "'");
        //     axis.send_estop_state(true);
        //     axis.last_estop_timestamp_ = time;
        //     return return_type::OK;
        // } else {
        //     RCLCPP_DEBUG_STREAM(rclcpp::get_logger("BetterODriveHardwareInterface"), 
        //         "EStop updated for axis '" << axis.node_id_ << "'");
        //     axis.send_estop_state(false);
        //     axis.last_estop_timestamp_ = time;
        // }
        // axis.estop_ = -1.0; // Reset command
        axis.send_estop_state(false);

        // Clear errors
        if (axis.clear_errors_cmd_ > 0.5) {
            RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("BetterODriveHardwareInterface"), clk, 1000, 
                "Clearing errors for axis '" << axis.node_id_ << "'");
            axis.send_clear_errors();
            axis.clear_errors_cmd_ = 0.0; // Consume command
        }

        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            float input_pos = axis.pos_setpoint_;
            float vel_ff = axis.vel_input_enabled_ ? axis.vel_setpoint_ : 0.0f;
            float torque_ff = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send_input_pos(input_pos, vel_ff, torque_ff);
        } else if (axis.vel_input_enabled_) {
            float input_vel = axis.vel_setpoint_;
            float input_torque_ff = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("BetterODriveHardwareInterface"), clk, 1000, 
                "Velocity for axis '" << axis.node_id_ << "' is: " << axis.vel_setpoint_);
            axis.send_input_vel(input_vel, input_torque_ff);
        } else if (axis.torque_input_enabled_) {
            float input_torque = axis.torque_setpoint_;
            axis.send_input_torque(input_torque);
        }

        bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

        if (axis.enable_ > 0.5 && any_enabled) {
            if (axis.axis_state_ == ODriveAxisState::AXIS_STATE_IDLE) {
                axis.send_axis_state(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        } else {
            if (axis.axis_state_ == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                axis.send_axis_state(ODriveAxisState::AXIS_STATE_IDLE);
            }
        }
    }

    return return_type::OK;
}

void BetterODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void Axis::send_axis_state(const ODriveAxisState& state) {
    Set_Axis_State_msg_t msg;
    msg.Axis_Requested_State = state;
    send(msg);
}

void Axis::send_controller_mode(const ODriveControlMode& control, const ODriveInputMode& input) {
    Set_Controller_Mode_msg_t msg;
    msg.Control_Mode = control;
    msg.Input_Mode = input;
    send(msg);
}

void Axis::send_clear_errors(const uint8_t& identify) {
    Clear_Errors_msg_t msg;
    msg.Identify = identify;
    send(msg);
}

void Axis::send_input_pos(const double &input_pos, const double &velocity_feed_forward, const double &torque_feed_forward)
{
    Set_Input_Pos_msg_t msg;
    msg.Input_Pos = input_pos / (2 * M_PI);
    msg.Vel_FF = velocity_feed_forward / (2 * M_PI);
    msg.Torque_FF = torque_feed_forward;
    send(msg);
}

void Axis::send_input_vel(const double &velocity, const double &torque_feed_forward)
{
    Set_Input_Vel_msg_t msg;
    msg.Input_Vel = velocity / (2 * M_PI);
    msg.Input_Torque_FF = torque_feed_forward;
    send(msg);
}

void Axis::send_input_torque(const double &torque)
{
    Set_Input_Torque_msg_t msg;
    msg.Input_Torque = torque;
    send(msg);
}

void Axis::send_estop_state(const bool &estop)
{
    Estop_msg_t msg;
    if (estop) {
        send(msg);
    } else {
        send_clear_errors();
    }
}

void Axis::send_absolute_position(const double &position)
{
    Set_Absolute_Position_msg_t msg;
    msg.Position = position / (2 * M_PI);
    send(msg);
}

void Axis::send_limits(const double &velocity_limit, const double &current_limit)
{
    Set_Limits_msg_t msg;
    msg.Velocity_Limit = velocity_limit / (2 * M_PI);
    msg.Current_Limit = current_limit;
    send(msg);
}

void Axis::send_trajectory_vel_limit(const double &limit)
{
    Set_Traj_Vel_Limit_msg_t msg;
    msg.Traj_Vel_Limit = limit / (2 * M_PI);
    send(msg);
}

void Axis::send_trajectory_accel_limits(const double &accel_limit, const double &decel_limit)
{
    Set_Traj_Accel_Limits_msg_t msg;
    msg.Traj_Accel_Limit = accel_limit / (2 * M_PI);
    msg.Traj_Decel_Limit = decel_limit / (2 * M_PI);
    send(msg);
}

void Axis::send_trajectory_inertia(const double &inertia)
{
    Set_Traj_Inertia_msg_t msg;
    msg.Traj_Inertia = inertia * (2 * M_PI);
    send(msg);
}

void Axis::on_can_msg(const rclcpp::Time& time, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("BetterODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
        case Get_Temperature_msg_t::cmd_id: {
            if (Get_Temperature_msg_t msg; try_decode(msg)) {
                fet_temperature_ = msg.FET_Temperature;
                motor_temperature_ = msg.Motor_Temperature;
            }
        } break;
        case Get_Bus_Voltage_Current_msg_t::cmd_id: {
            if (Get_Bus_Voltage_Current_msg_t msg; try_decode(msg)) {
                bus_voltage_ = msg.Bus_Voltage;
                bus_current_ = msg.Bus_Current;
            }
        } break;
        case Get_Error_msg_t::cmd_id: {
            if (Get_Error_msg_t msg; try_decode(msg)) {
                active_errors_ = msg.Active_Errors;
                disarm_reason_ = msg.Disarm_Reason;
            }
        } break;
        case Get_Iq_msg_t::cmd_id: {
            if (Get_Iq_msg_t msg; try_decode(msg)) {
                iq_measured_ = msg.Iq_Measured;
                iq_setpoint_ = msg.Iq_Setpoint;
            }
        } break;
        case Get_Powers_msg_t::cmd_id: {
            if (Get_Powers_msg_t msg; try_decode(msg)) {
                electrical_power_ = msg.Electrical_Power;
                mechanical_power_ = msg.Mechanical_Power;
            }
        } break;
        case Get_Version_msg_t::cmd_id: {
            if (Get_Version_msg_t msg; try_decode(msg)) {
                protocol_version_ = msg.Protocol_Version;
                hw_version_major_ = msg.Hw_Version_Major;
                hw_version_minor_ = msg.Hw_Version_Minor;
                hw_version_variant_ = msg.Hw_Version_Variant;
                fw_version_major_ = msg.Fw_Version_Major;
                fw_version_minor_ = msg.Fw_Version_Minor;
                fw_version_revision_ = msg.Fw_Version_Revision;
                fw_version_unreleased_ = msg.Fw_Version_Unreleased;
            }
        } break;
        case Heartbeat_msg_t::cmd_id: {
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                axis_error_ = msg.Axis_Error;
                axis_state_ = msg.Axis_State;
                procedure_result_ = msg.Procedure_Result;
                trajectory_done_flag_ = msg.Trajectory_Done_Flag;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::BetterODriveHardwareInterface, hardware_interface::SystemInterface)

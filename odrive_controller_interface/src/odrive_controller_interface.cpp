#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "odrive_controller_interface/odrive_controller_interface_parameters.hpp"


namespace odrive_controller_interface {

class Axis;

using trigger_command = double;
using uint32_state = double;
using uint8_state = double;

class BetterODriveControllerInterface : public controller_interface::ControllerInterface {
    using TriggerSrv = std_srvs::srv::Trigger;
    using BoolMsg = std_msgs::msg::Bool;

public:
  BetterODriveControllerInterface();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Parameters from ROS for diff_drive_controller
//   std::shared_ptr<ParamListener> param_listener_;
//   Params params_;

private:
    std::vector<Axis> axes_;
};

struct Axis {
    Axis() {}

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0; // [rad]
    double vel_setpoint_ = 0.0; // [rad/s]
    double torque_setpoint_ = 0.0; // [Nm]
    double estop_ = -1.0;
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
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    uint32_state active_errors_ = 0;
    uint32_state disarm_reason_ = 0;
    double bus_voltage_ = NAN; // [V]
    double bus_current_ = NAN; // [A]
    double electrical_power_ = NAN; // [W]
    double mechanical_power_ = NAN; // [W]


    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    // /**
    //  * @brief Set the axis state
    //  * 
    //  * @param state Requested axis state
    //  */
    // void send_axis_state(const ODriveAxisState& state);
    // /**
    //  * @brief Set the controller mode 
    //  * 
    //  * @param control Requested controller mode
    //  * @param input Requested input mode
    //  */
    // void send_controller_mode(const ODriveControlMode& control, const ODriveInputMode& input);
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

};


} // namespace odrive_controller_interface

using namespace odrive_controller_interface;



PLUGINLIB_EXPORT_CLASS(odrive_controller_interface::BetterODriveControllerInterface, controller_interface::ControllerInterface)

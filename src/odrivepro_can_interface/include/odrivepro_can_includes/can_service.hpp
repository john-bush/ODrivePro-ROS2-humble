#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

// #include "odrivepro_ros2_can/msg/odrive_status.hpp"

#include "srv_and_msg/srv/odrive_estop.hpp"
#include "srv_and_msg/srv/get_motor_error.hpp"
#include "srv_and_msg/srv/get_encoder_error.hpp"
#include "srv_and_msg/srv/set_axis_node_id.hpp"
#include "srv_and_msg/srv/set_axis_requested_state.hpp"
#include "srv_and_msg/srv/set_axis_startup_config.hpp"
#include "srv_and_msg/srv/get_encoder_estimates.hpp"
#include "srv_and_msg/srv/get_encoder_count.hpp"
#include "srv_and_msg/srv/set_controller_modes.hpp"
#include "srv_and_msg/srv/set_input_pos.hpp"
#include "srv_and_msg/srv/set_input_vel.hpp"
#include "srv_and_msg/srv/set_input_torque.hpp"
#include "srv_and_msg/srv/set_vel_limit.hpp"
#include "srv_and_msg/srv/start_anticogging.hpp"
#include "srv_and_msg/srv/set_traj_vel_limit.hpp"
#include "srv_and_msg/srv/set_traj_accel_limits.hpp"
#include "srv_and_msg/srv/set_traj_inertia.hpp"
#include "srv_and_msg/srv/get_iq.hpp"
#include "srv_and_msg/srv/reset_odrive.hpp"
#include "srv_and_msg/srv/get_vbus_voltage.hpp"
#include "srv_and_msg/srv/clear_errors.hpp"
#include "srv_and_msg/srv/get_temperature.hpp"
#include "srv_and_msg/srv/set_absolute_pos.hpp"
#include "srv_and_msg/srv/set_pos_gain.hpp"
#include "srv_and_msg/srv/set_vel_gains.hpp"



class CanService : public rclcpp::Node
{
public:
    CanService(/* args */);
    ~CanService();

private:
    rclcpp::Service<odrivepro_ros2_can::srv::OdriveEstop>::SharedPtr service_odrive_estop_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetMotorError>::SharedPtr service_get_motor_error_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetEncoderError>::SharedPtr service_get_encoder_error_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetAxisNodeId>::SharedPtr service_set_axis_node_id_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetAxisRequestedState>::SharedPtr service_set_axis_requested_state_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetAxisStartupConfig>::SharedPtr service_set_axis_startup_config_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetEncoderEstimates>::SharedPtr service_get_encoder_estimates_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetEncoderCount>::SharedPtr service_get_encoder_count_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetControllerModes>::SharedPtr service_set_controller_modes_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetInputPos>::SharedPtr service_set_input_pos_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetInputVel>::SharedPtr service_set_input_vel_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetInputTorque>::SharedPtr service_set_input_torque_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetVelLimit>::SharedPtr service_set_vel_limit_;
    rclcpp::Service<odrivepro_ros2_can::srv::StartAnticogging>::SharedPtr service_start_anticogging_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetTrajVelLimit>::SharedPtr service_set_traj_vel_limit_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetTrajAccelLimits>::SharedPtr service_set_traj_accel_limits_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetTrajInertia>::SharedPtr service_set_traj_inertia_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetIq>::SharedPtr service_get_iq_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetTemperature>::SharedPtr service_get_temperature_;
    rclcpp::Service<odrivepro_ros2_can::srv::ResetOdrive>::SharedPtr service_reset_odrive_;
    rclcpp::Service<odrivepro_ros2_can::srv::GetVbusVoltage>::SharedPtr service_get_vbus_voltage_;
    rclcpp::Service<odrivepro_ros2_can::srv::ClearErrors>::SharedPtr service_clear_errors_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetAbsolutePos>::SharedPtr service_set_absolute_pos_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetPosGain>::SharedPtr service_set_pos_gain_;
    rclcpp::Service<odrivepro_ros2_can::srv::SetVelGains>::SharedPtr service_set_vel_gains_;



    SocketcanInterface socket_get_motor_error_;
    SocketcanInterface socket_get_encoder_error_;
    SocketcanInterface socket_get_encoder_estimates_;
    SocketcanInterface socket_get_encoder_count_;
    SocketcanInterface socket_get_iq_;
    SocketcanInterface socket_get_temperature_;
    SocketcanInterface socket_get_vbus_voltage_;
    SocketcanInterface socket_generic_write_;

    void odrive_estop_callback(const std::shared_ptr<odrivepro_ros2_can::srv::OdriveEstop::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::OdriveEstop::Response> response);
    void get_motor_error_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetMotorError::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetMotorError::Response> response);
    void get_encoder_error_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderError::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderError::Response> response);
    void set_axis_node_id_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetAxisNodeId::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetAxisNodeId::Response> response);
    void set_axis_requested_state_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetAxisRequestedState::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetAxisRequestedState::Response> response);
    void set_axis_startup_config_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetAxisStartupConfig::Response> response);
    void get_encoder_estimates_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderEstimates::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderEstimates::Response> response);
    void get_encoder_count_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderCount::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetEncoderCount::Response> response);
    void set_controller_modes_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetControllerModes::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetControllerModes::Response> response);
    void set_input_pos_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetInputPos::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetInputPos::Response> response);
    void set_input_vel_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetInputVel::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetInputVel::Response> response);
    void set_input_torque_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetInputTorque::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetInputTorque::Response> response);
    void set_vel_limit_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetVelLimit::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetVelLimit::Response> response);
    void start_anticogging_callback(const std::shared_ptr<odrivepro_ros2_can::srv::StartAnticogging::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::StartAnticogging::Response> response);
    void set_traj_vel_limit_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetTrajVelLimit::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetTrajVelLimit::Response> response);
    void set_traj_accel_limits_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetTrajAccelLimits::Response> response);
    void set_traj_inertia_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetTrajInertia::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetTrajInertia::Response> response);
    void get_iq_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetIq::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetIq::Response> response);
    void get_temperature_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetTemperature::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetTemperature::Response> response);
    void reset_odrive_callback(const std::shared_ptr<odrivepro_ros2_can::srv::ResetOdrive::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::ResetOdrive::Response> response);
    void get_vbus_voltage_callback(const std::shared_ptr<odrivepro_ros2_can::srv::GetVbusVoltage::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::GetVbusVoltage::Response> response);
    void clear_errors_callback(const std::shared_ptr<odrivepro_ros2_can::srv::ClearErrors::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::ClearErrors::Response> response);
    void set_absolute_pos_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetAbsolutePos::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetAbsolutePos::Response> response);
    void set_pos_gain_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetPosGain::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetPosGain::Response> response);
    void set_vel_gains_callback(const std::shared_ptr<odrivepro_ros2_can::srv::SetVelGains::Request> request, std::shared_ptr<odrivepro_ros2_can::srv::SetVelGains::Response> response);
};
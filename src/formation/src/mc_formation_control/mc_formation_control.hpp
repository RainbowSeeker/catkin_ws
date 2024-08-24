#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/formation_cross.hpp>
#include <form_msgs/msg/uav_command.hpp>

#include "formation/parameter_manager.hpp"
#include "control_toolbox/pid.hpp"


#define SILSIM		1	// Software in the loop simulation
#define REAL		2	// Real flight

#define SIM_MODE	SILSIM

#define PHASE_SINGLE	0
#define PHASE_FORMATION	1
#define PHASE_UNVALID	-1
#define PHASE_DEFAULT	PHASE_FORMATION

#define ROS_ZERO_TIME   0, 0, RCL_ROS_TIME

using namespace std::chrono_literals;

namespace formation {
class MulticopterFormationControl : public rclcpp::Node {

public:
    MulticopterFormationControl(const std::string& node_name, std::chrono::milliseconds control_period = 20ms);
    ~MulticopterFormationControl() = default;

private:
    inline uint64_t absolute_time() {
        return get_clock()->now().nanoseconds() / 1e3; // [us]
    }

    void timer_callback();
	bool formation_preprocess();
	void formation_step();
	void publish_trajectory_setpoint(float velocity[3], float yawspeed);
	void fms_step();

    rclcpp::TimerBase::SharedPtr _timer;

    // Subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr    _local_pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr         _att_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr           _vehicle_status_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr    _form_pos_sub[3];
    rclcpp::Subscription<form_msgs::msg::UavCommand>::SharedPtr             _command_sub[3];

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr         _trajectory_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr        _offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr             _vehicle_command_pub;

    // Parameters
    rclcpp::Parameter      _param_hgt_sp{"mc_hgt_sp", 5.0};
    rclcpp::Parameter      _param_hgt_kp{"mc_hgt_kp", 1.0};
    rclcpp::Parameter      _param_hgt_ki{"mc_hgt_ki", 0.1};
    
    inline void parameters_declare()
    {
        ParameterManager::add_parameter(&_param_hgt_sp);
        ParameterManager::add_parameter(&_param_hgt_kp);
        ParameterManager::add_parameter(&_param_hgt_ki);
        ParameterManager::attach(this).declare_parameters();
    }

    // Deprecated. Parameters are updated in parameter_manager.hpp
    inline void parameters_update()
    {
        _hgt_ctrl.setGains(_param_hgt_kp.as_double(), _param_hgt_ki.as_double(), 0.0, 0.5, -0.5);
    }

    // control_toolbox
    control_toolbox::Pid _hgt_ctrl {_param_hgt_kp.as_double(), _param_hgt_ki.as_double(), 0.0, 0.0, -0.0};


    px4_msgs::msg::FormationCross		_formation_cross{};
    px4_msgs::msg::VehicleLocalPosition _local_pos{};
	px4_msgs::msg::VehicleAttitude	    _att{};
    px4_msgs::msg::VehicleStatus	    _vehicle_status{};

    int32_t _test_phase{PHASE_DEFAULT};	// determine the running type of uavs. 0: single, 1: formation

    rclcpp::Time _first_ready_time{ROS_ZERO_TIME};
    uint64_t _control_interval; // [ns]

    int _uav_id{0};
	rclcpp::Duration running_time{0, 0};

    struct {
        float velocity[3];
        float yawspeed;
        uint64_t timestamp;
    } _fms_out;
};
} // namespace formation
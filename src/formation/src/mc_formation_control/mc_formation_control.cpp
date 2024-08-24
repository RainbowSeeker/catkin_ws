#include "mc_formation_control.hpp"
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace Eigen;

namespace formation {

MulticopterFormationControl::MulticopterFormationControl(const std::string& node_name, std::chrono::milliseconds control_period) : 
    Node(node_name), _control_interval(control_period / 1ms * 1e6) // [ns]
{
    std::string topic_ns{""};

    if (std::isdigit(node_name.back())) {
        topic_ns = '/' + std::string("px4_")+ node_name.back();
        _uav_id = node_name.back() - '1';

        if (_uav_id < 0 || _uav_id >= 3)
            throw std::invalid_argument("Invalid uav id");
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
    _local_pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        topic_ns + "/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            _local_pos = *msg;
        });

    _att_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        topic_ns + "/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            _att = *msg;
        });

    _vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        topic_ns + "/fmu/out/vehicle_status", qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            _vehicle_status = *msg;
        });

    _trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        topic_ns + "/fmu/in/trajectory_setpoint", 10);

    _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        topic_ns + "/fmu/in/offboard_control_mode", 10);

    _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        topic_ns + "/fmu/in/vehicle_command", 10);
        
    // Subscribe formation cross
    for (int i = 0; i < 3; i++)
    {
        _form_pos_sub[i] = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        std::string("px4_") + (char)('1' + i) + "/fmu/out/vehicle_local_position", qos,
        [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            _formation_cross.x[i] = msg->x;
            _formation_cross.y[i] = msg->y;
            _formation_cross.h[i] = -msg->z;
            _formation_cross.vn[i] = msg->vx;
            _formation_cross.ve[i] = msg->vy;
            _formation_cross.vd[i] = msg->vz;
            _formation_cross.time_usec[i] = msg->timestamp;
        });
    }
    
    parameters_declare();
    
    RCLCPP_INFO(this->get_logger(), "Formation node for %s started.", topic_ns.c_str());
    _timer = this->create_wall_timer(control_period, std::bind(&MulticopterFormationControl::timer_callback, this));
}


void MulticopterFormationControl::timer_callback() 
{
	/* only run controller if cross data ready */
	if (formation_preprocess())
	{
		if (_first_ready_time == rclcpp::Time(ROS_ZERO_TIME))
        {
            _first_ready_time = get_clock()->now();
        }

		formation_step();

        running_time = get_clock()->now() - _first_ready_time;
	}
	else
	{
		_first_ready_time = rclcpp::Time(ROS_ZERO_TIME);
	}
}

void MulticopterFormationControl::publish_trajectory_setpoint(float velocity[3], float yawspeed) 
{
    px4_msgs::msg::OffboardControlMode ocm{};
	ocm.position = false;
	ocm.velocity = true;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.actuator = false;
	ocm.timestamp = absolute_time();
	_offboard_control_mode_pub->publish(ocm);

	px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position = {NAN, NAN, NAN};
    setpoint.velocity = {velocity[0], velocity[1], velocity[2]};
    setpoint.yawspeed = yawspeed;
    setpoint.timestamp = absolute_time();
	_trajectory_setpoint_pub->publish(setpoint);
}

void MulticopterFormationControl::formation_step() 
{
    parameters_update();
	/* Run attitude controllers */
	fms_step();
	/* Publish the attitude setpoint for analysis once available */
	publish_trajectory_setpoint(_fms_out.velocity, _fms_out.yawspeed);
}

void MulticopterFormationControl::fms_step() 
{
    // control interval [ns]
    const uint64_t dt = _control_interval;

    // height hold
    double vh_cmd = _hgt_ctrl.computeCommand(_param_hgt_sp.as_double() + _local_pos.z, dt);

    // formation control
    Vector3d pos_err{0, 0, 0};
    Vector3d vel_sp {1.0, 0, -vh_cmd};

    Matrix3d rel_x; 
    rel_x << 0.0, -20.0, -20.0, 
             20.0, 0.0, 0.0, 
             20.0, 0.0, 0.0;
    Matrix3d rel_y; 
    rel_y << 0.0, -20.0, 20.0, 
             20.0, 0.0, 40.0, 
             -20.0, -40.0, 0.0;
    Matrix3d rel_z = Matrix3d::Zero();
    for (int i = 0; i < 3; i++)
    {
        pos_err += Vector3d{_formation_cross.x[i] + rel_x(i, _uav_id), _formation_cross.y[i] + rel_y(i, _uav_id), -_formation_cross.h[i] + rel_z(i, _uav_id)} / 3.0;
        // vel_sp  += Vector3d{_formation_cross.vn[i], _formation_cross.ve[i], _formation_cross.vd[i]} / 3.0;
    }
    pos_err -= Vector3d{_local_pos.x, _local_pos.y, _local_pos.z};
    vel_sp += pos_err * 0.5;

    _fms_out.velocity[0] = (float)vel_sp[0];
    _fms_out.velocity[1] = (float)vel_sp[1];
    _fms_out.velocity[2] = (float)vel_sp[2];
    _fms_out.yawspeed = 0.0f;
    
	RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
			"performance: x_err: %f, y_err: %f, z_err: %f", pos_err(0), pos_err(1), pos_err(2));
    
}

bool MulticopterFormationControl::formation_preprocess() 
{
    // check list 1: is flying
    bool is_flying = (_vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) &&
            (_vehicle_status.nav_state > px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL);

    if (!is_flying)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "not flying, nav_state: %d, arming_state: %d", _vehicle_status.nav_state, _vehicle_status.arming_state);
        return false;
    }

#if SIM_MODE == SILSIM
	static rclcpp::Time takeoff_time{get_clock()->now()};
	if (get_clock()->now() - takeoff_time < 8s)
	{
		RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "wait for takeoff.");
		return false;
	}
#endif
	// check list 2: switch to formation mode
	bool is_offboard = _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
	if (!is_offboard)
	{
		px4_msgs::msg::VehicleCommand cmd{};
		cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
		cmd.param1 = 1; // base mode
		cmd.param2 = 6;
        cmd.target_system = _vehicle_status.system_id;
		cmd.source_system = _vehicle_status.system_id;
        cmd.target_component = _vehicle_status.component_id;
        cmd.source_component = _vehicle_status.component_id;
        cmd.from_external = true;
        cmd.timestamp = absolute_time();
		_vehicle_command_pub->publish(cmd);

		formation_step();

		RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Try to switch to formation mode.");
		return false;
	}

	// check list over.
	return true;
}


} // namespace formation

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <node_name>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<formation::MulticopterFormationControl>(argv[1], 40ms));

    rclcpp::shutdown();
    return 0;
}
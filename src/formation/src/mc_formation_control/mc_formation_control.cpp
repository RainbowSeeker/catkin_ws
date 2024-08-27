#include "mc_formation_control.hpp"
#include "formation/utils.hpp"
#include "px4_ros_com/frame_transforms.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace Eigen;

namespace formation {

MulticopterFormationControl::MulticopterFormationControl(const std::string& node_name, std::chrono::milliseconds control_period) : 
    Node(node_name), _control_interval(control_period / 1ms * 1e6) // [ns]
{
    parameters_declare();

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
            _last_fmuout_time = get_clock()->now();
        });

    _att_sub = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        topic_ns + "/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            using namespace px4_ros_com::frame_transforms::utils::quaternion;
            _att = *msg;
            _yaw = quaternion_get_yaw(array_to_eigen_quat(_att.q));
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
    _test_phase = _param_test_phase.as_string() == "formation" ? PHASE_FORMATION : PHASE_SINGLE;

    if (_test_phase == PHASE_FORMATION)
    {
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
                _last_cross_time[i] = get_clock()->now();
            });
        }
    }

    _command_sub = this->create_subscription<form_msgs::msg::UavCommand>(
        topic_ns + "/fmu/in/uav_command", qos,
        std::bind(&MulticopterFormationControl::handle_command, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Formation node for %s started. Phase: %s", topic_ns.c_str(), _param_test_phase.as_string().c_str());
    _timer = this->create_wall_timer(control_period, std::bind(&MulticopterFormationControl::timer_callback, this));
}


void MulticopterFormationControl::timer_callback() 
{
	/* only run controller if cross data ready */
	if (formation_preprocess())
	{
        formation_enter();

		formation_step();

        _running_time = get_clock()->now() - _first_ready_time;
	}
	else
	{
        formation_exit();
	}
}

bool MulticopterFormationControl::formation_preprocess()
{
#if SIM_MODE == REAL
    // check list 0: is ready
    if (_stop)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Wait for GCS.");
        return false;
    }
#endif
    // check list 1: px4 data ready
    if (!uav_is_active())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "Fmu data lost.");
        return false;
    }

    if (_test_phase == PHASE_FORMATION)
    {
        // check list 2: cross data ready
        for (int i = 0; i < 3; i++)
        {
            if (get_clock()->now() - _last_cross_time[i] > 1s)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Wait for cross data from id: %d.", i);
                return false;
            }
        }
    }

    // check list 3: manual control
    bool is_manual = (_vehicle_status.nav_state > px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL && 
            _vehicle_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);

    if (!is_manual)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "cannot control, nav_state: %d, arming_state: %d", _vehicle_status.nav_state, _vehicle_status.arming_state);
        return false;
    }

	// check list 4: switch to formation mode
	bool is_offboard = _vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED && 
                        _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
	if (!is_offboard)
	{
		// switch to offboard mode
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);

		formation_step();

		RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Try to switch to formation mode.");
		return false;
	}

	// check list over.
	return true;
}

void MulticopterFormationControl::formation_enter()
{
    if (_first_ready_time == rclcpp::Time(ROS_ZERO_TIME))
    {
        _first_ready_time = get_clock()->now();
    }
}

void MulticopterFormationControl::formation_exit()
{
    // when uav firstly get ready and exit, enter state of emergency.
    if (_first_ready_time != rclcpp::Time(ROS_ZERO_TIME))
    {
        _is_emergency = true;
    }

    if (_is_emergency && uav_is_active())
    {
        if (_vehicle_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER && 
            _vehicle_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL) 
        {
            // switch to hold mode
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                    "emergency: Try to switch to hold mode.");
        } else 
        {
            _is_emergency = false;
            RCLCPP_INFO(this->get_logger(), "emergency: Succeed in switching to hold mode.");
        }
    }

    _first_ready_time = rclcpp::Time(ROS_ZERO_TIME);
}

void MulticopterFormationControl::formation_step() 
{
    parameters_update();
	/* Run attitude controllers */
	fms_step();
	/* Publish the attitude setpoint for analysis once available */
	publish_trajectory_setpoint(_fms_out.velocity, _fms_out.yaw);
}

void MulticopterFormationControl::fms_step() 
{
    // control interval [ns]
    const uint64_t dt = _control_interval;

    // height hold
    double vh_cmd = _hgt_ctrl.computeCommand(_param_hgt_sp.as_double() + _local_pos.z, dt);

    // yaw hold
    const double yaw_cmd = math::radians(_param_yaw_sp.as_double());

    // formation control
    Vector3d pos_err{0, 0, 0};
    Vector3d vel_sp {1.0, 0, 0};

    if (_test_phase == PHASE_SINGLE)
    {
        // follow yaw.
        Matrix3d Rz;
        Rz << cos(_yaw), -sin(_yaw),  0,
              sin(_yaw),  cos(_yaw),  0,
              0,                  0,  1;

        vel_sp = Rz * vel_sp;
        vel_sp[2] += -vh_cmd;
    }
    else if (_test_phase == PHASE_FORMATION)
    {
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
        vel_sp[2] += -vh_cmd;
    }
    
    // output limit
    vel_sp[0] = math::constrain(vel_sp[0], -1.0, 1.0);
    vel_sp[1] = math::constrain(vel_sp[1], -1.0, 1.0);
    vel_sp[2] = math::constrain(vel_sp[2], -1.0, 1.0);

    _fms_out.velocity[0] = (float)vel_sp[0];
    _fms_out.velocity[1] = (float)vel_sp[1];
    _fms_out.velocity[2] = (float)vel_sp[2];
    _fms_out.yaw = (float)yaw_cmd;
    
    if (_test_phase == PHASE_SINGLE)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
            "command vn: %f, ve: %f, vd: %f, yaw_cmd: %f deg", vel_sp[0], vel_sp[1], vel_sp[2], math::degrees(yaw_cmd));
    }
    else if (_test_phase == PHASE_FORMATION)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
            "formation: x_err: %f, y_err: %f, z_err: %f", pos_err(0), pos_err(1), pos_err(2));
    }
}

void MulticopterFormationControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) 
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;
    cmd.target_system = _vehicle_status.system_id;
    cmd.source_system = _vehicle_status.system_id;
    cmd.target_component = _vehicle_status.component_id;
    cmd.source_component = _vehicle_status.component_id;
    cmd.from_external = true;
    cmd.timestamp = absolute_time();
    _vehicle_command_pub->publish(cmd);
}

void MulticopterFormationControl::publish_trajectory_setpoint(float velocity[3], float yaw) 
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
    setpoint.yaw = yaw;
    setpoint.timestamp = absolute_time();
	_trajectory_setpoint_pub->publish(setpoint);
}


void MulticopterFormationControl::handle_command(const form_msgs::msg::UavCommand::SharedPtr msg)
{
    using namespace form_msgs::msg;

    switch (msg->command)
    {
    case UavCommand::UAV_CMD_FORM_START:
        _stop = false;
        break;
    case UavCommand::UAV_CMD_FORM_STOP:
        _stop = true;
        break;
    default:
        break;
    }

    RCLCPP_INFO(this->get_logger(), "Received command: %d, param: %.3f %.3f", msg->command, msg->param1, msg->param2);
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
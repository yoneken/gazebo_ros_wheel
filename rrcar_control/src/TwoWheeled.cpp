#include <rrcar_control/TwoWheeled.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <iostream>

namespace rrcar_control
{

//TwoWheeled::TwoWheeled(const ros::NodeHandle& nh)
TwoWheeled::TwoWheeled(ros::NodeHandle nh)
: pos_({0}), vel_({0}), eff_({0}), cmd_({0}),
  root_nh_(nh), tachometer_({0.0})
{
	// load parameters
	std::string l_wheel_drv_topic_name, r_wheel_drv_topic_name;
	std::string l_wheel_sensor_topic_name, r_wheel_sensor_topic_name;
	root_nh_.param<std::string>("l_wheel_drv_topic", l_wheel_drv_topic_name, "wheel_left/commands/motor/duty_cycle");
	root_nh_.param<std::string>("r_wheel_drv_topic", r_wheel_drv_topic_name, "wheel_right/commands/motor/duty_cycle");
	root_nh_.param<std::string>("l_wheel_sensor_topic", l_wheel_sensor_topic_name, "wheel_left/sensors/core");
	root_nh_.param<std::string>("r_wheel_sensor_topic", r_wheel_sensor_topic_name, "wheel_right/sensors/core");
	root_nh_.param("num_motor_poles", NUM_MOTOR_POLES, 83);

	// init pub/sub
	duty_l_pub_ = root_nh_.advertise<std_msgs::Float64>(l_wheel_drv_topic_name, 10);
	duty_r_pub_ = root_nh_.advertise<std_msgs::Float64>(r_wheel_drv_topic_name, 10);
	sensor_l_sub_ = root_nh_.subscribe(l_wheel_sensor_topic_name, 10, &TwoWheeled::sensor_l_Callback, this);
	sensor_r_sub_ = root_nh_.subscribe(r_wheel_sensor_topic_name, 10, &TwoWheeled::sensor_r_Callback, this);

	// connect and register joint_state_insterfaces
	hardware_interface::JointStateHandle state_handle0("wheel_left_joint", &pos_[0], &vel_[0], &eff_[0]);
	jnt_state_interface.registerHandle(state_handle0);

	hardware_interface::JointStateHandle state_handle1("wheel_right_joint", &pos_[1], &vel_[1], &eff_[1]);
	jnt_state_interface.registerHandle(state_handle1);

	registerInterface(&jnt_state_interface);

	// connect and register joint_effort_interfaces
	hardware_interface::JointHandle eff_handle0(jnt_state_interface.getHandle("wheel_left_joint"), &cmd_[0]);
	jnt_eff_interface.registerHandle(eff_handle0);

	hardware_interface::JointHandle eff_handle1(jnt_state_interface.getHandle("wheel_right_joint"), &cmd_[1]);
	jnt_eff_interface.registerHandle(eff_handle1);

	registerInterface(&jnt_eff_interface);
}

void TwoWheeled::read(ros::Time time, ros::Duration period)
{
	pos_[0] = tachometer_[0] / NUM_MOTOR_POLES * 2 * M_PI;
	pos_[1] = tachometer_[1] / NUM_MOTOR_POLES * 2 * M_PI;
}

void TwoWheeled::write(ros::Time time, ros::Duration period)
{
	// command each motor
	std_msgs::Float64::Ptr cmd_l(new std_msgs::Float64);
	std_msgs::Float64::Ptr cmd_r(new std_msgs::Float64);
	cmd_l->data = cmd_[0];
	cmd_r->data = cmd_[1];
	duty_l_pub_.publish(cmd_l);
	duty_r_pub_.publish(cmd_r);
}

void TwoWheeled::sensor_l_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
	tachometer_[0] = state->state.distance_traveled;
}

void TwoWheeled::sensor_r_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
	tachometer_[1] = state->state.distance_traveled;
}

} // namespace rrcar_control


#include <rrcar_control/TwoWheeled.h>
#include <std_msgs/Float64.h>

//const unsigned short TwoWheeled::NUM_MOTOR_POLES = 84;
//const double TwoWheeled::RAD_WHEEL = 5.0;

//TwoWheeled::TwoWheeled(const ros::NodeHandle& nh)
TwoWheeled::TwoWheeled(ros::NodeHandle nh)
: pos_({0}), vel_({0}), eff_({0}), cmd_({0}),
  tachometer_({0.0})
{
	root_nh_ = nh;

	// init pub/sub
	ros::Publisher duty_cycle_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
	ros::Subscriber state_sub_ = nh.subscribe("sensors/core", 10, &TwoWheeled::stateCallback, this);

	// connect and register joint_state_insterfaces
	hardware_interface::JointStateHandle state_handle0("wheel_left_joint", &pos_[0], &vel_[0], &eff_[0]);
	jnt_state_interface.registerHandle(state_handle0);

	hardware_interface::JointStateHandle state_handle1("wheel_right_joint", &pos_[1], &vel_[1], &eff_[1]);
	jnt_state_interface.registerHandle(state_handle1);

	registerInterface(&jnt_state_interface);

	// vonnect and register joint_effort_interfaces
	hardware_interface::JointHandle eff_handle0(jnt_state_interface.getHandle("wheel_left_joint"), &cmd_[0]);
	jnt_eff_interface.registerHandle(eff_handle0);

	hardware_interface::JointHandle eff_handle1(jnt_state_interface.getHandle("wheel_right_joint"), &cmd_[1]);
	jnt_eff_interface.registerHandle(eff_handle1);

	registerInterface(&jnt_eff_interface);
}

void TwoWheeled::read(ros::Time time, ros::Duration period)
{
	pos_[0] = tachometer_[0];
	pos_[1] = tachometer_[1];
}

void TwoWheeled::write(ros::Time time, ros::Duration period)
{
	std_msgs::Float64::Ptr duty_msg(new std_msgs::Float64);
	duty_msg->data = cmd_[0];
	duty_cycle_pub_.publish(duty_msg);
}

bool TwoWheeled::setup(void)
{
	bool res = false;

	return res;
}

void TwoWheeled::shutdown(void)
{
}

void TwoWheeled::stateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
	tachometer_[0] = state->state.distance_traveled;
}


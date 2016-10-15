#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <vesc_msgs/VescStateStamped.h>

namespace rrcar_control
{

class TwoWheeled : public hardware_interface::RobotHW
{
public:
	//TwoWheeled(const ros::NodeHandle& nh=ros::NodeHandle());
	TwoWheeled(ros::NodeHandle nh);

	inline std_msgs::TimeConstPtr getTime() const
	{
		std_msgs::TimePtr now(new std_msgs::Time());
		now->data = ros::Time::now();
		return now;
	}

	inline std_msgs::DurationConstPtr getPeriod() const
	{ 
		std_msgs::DurationPtr dt(new std_msgs::Duration());
		dt->data = ros::Duration(0.01);
		return dt;
	}

	void read(ros::Time, ros::Duration);
	void read(const std_msgs::TimeConstPtr&, const std_msgs::DurationConstPtr&);
	void write(ros::Time, ros::Duration);
	void write(const std_msgs::TimeConstPtr&, const std_msgs::DurationConstPtr&);

	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double pos_[2], vel_[2], eff_[2], cmd_[2];
	double l_pos_[2];

protected:
	int NUM_MOTOR_POLES;
	double WHEEL_RADIUS;

	ros::NodeHandle root_nh_;
	ros::Publisher duty_l_pub_, duty_r_pub_;
	ros::Subscriber sensor_l_sub_, sensor_r_sub_;
	double tachometer_[2];

	void sensor_l_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
	void sensor_r_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
};

} // namespace rrcar_control


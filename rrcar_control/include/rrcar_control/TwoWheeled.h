#include <ros/ros.h>
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

	ros::Time getTime() const { return ros::Time::now(); }
	ros::Duration getPeriod() const { return ros::Duration(0.01); }

	void read(ros::Time, ros::Duration);
	void write(ros::Time, ros::Duration);

	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::EffortJointInterface jnt_eff_interface;
	double pos_[3], vel_[3], eff_[3], cmd_[3];

protected:
	//static const unsigned short NUM_MOTOR_POLES;
	//static const double RAD_WHEEL;

	ros::NodeHandle root_nh_;
	ros::Publisher duty_l_pub_, duty_r_pub_;
	ros::Subscriber sensor_l_sub_, sensor_r_sub_;
	double tachometer_[2];

	void sensor_l_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
	void sensor_r_Callback(const vesc_msgs::VescStateStamped::ConstPtr& state);
};

} // namespace rrcar_control


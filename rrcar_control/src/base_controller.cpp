#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cmath>

static int CONTROL_HZ = 50;
static double TRACK_WIDTH = 0.22;
static double WHEEL_RADIUS = 0.05;
static int MOTOR_POLES = 83;

static double vx_= 0.0, vy_= 0.0, omega_= 0.0;
static double x_= 0.0, y_= 0.0, theta_= 0.0;
static double dx_= 0.0, dy_= 0.0, dtheta_= 0.0;
static double duty_left = 0.0, duty_right = 0.0;
static long erpml = 0, erpmr = 0;
static double cmd_vx = 0.0, cmd_yaw = 0.0;

void cmd_callback(const geometry_msgs::Twist &msg)
{
	cmd_vx = msg.linear.x;
	cmd_yaw = msg.angular.z;
}

void update_odometry(double dt)
{
	double v, vl, vr;
	vl = WHEEL_RADIUS * erpml / MOTOR_POLES * 2. * M_PI;
	vr = WHEEL_RADIUS * erpmr / MOTOR_POLES * 2. * M_PI;

	omega_ = (vr - vl) / TRACK_WIDTH;
	v = (vr + vl) / 2.;

	dx_ = v * cos(theta_);
	dy_ = v * sin(theta_);
	dtheta_ = omega_;
}

void publish_odom_tf(void)
{
}

void publish_odom_topic(void)
{
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "BaseController");
	ros::NodeHandle nh;

	// load parameters
	std::string odom_topic_name, cmd_topic_name;
	std::string l_wheel_topic_name, r_wheel_topic_name;
	nh.param<std::string>("odom_topic", odom_topic_name, "odom");
	nh.param<std::string>("cmd_topic", cmd_topic_name, "cmd_vel");
	nh.param<std::string>("l_wheel_topic", l_wheel_topic_name, "wheel_left_effort_controller/command");
	nh.param<std::string>("r_wheel_topic", r_wheel_topic_name, "wheel_right_effort_controller/command");

	nh.param("control_hz", CONTROL_HZ, 50);
	nh.param("track_width", TRACK_WIDTH, 0.22);
	nh.param("wheel_radius", WHEEL_RADIUS, 0.05);
	nh.param("motor_poles", MOTOR_POLES, 83);

	tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>(odom_topic_name, 10);
	ros::Subscriber sub = nh.subscribe(cmd_topic_name, 10, cmd_callback);
	ros::Publisher pub_l_wheel = nh.advertise<std_msgs::Float64>(l_wheel_topic_name, 10);
	ros::Publisher pub_r_wheel = nh.advertise<std_msgs::Float64>(r_wheel_topic_name, 10);

	ros::Time current_time;
	ros::Time last_time = ros::Time::now();
	ros::Rate loop(CONTROL_HZ);

	while(ros::ok()){
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		update_odometry(dt);
		publish_odom_tf();
		publish_odom_topic();

		// command each motor
		std_msgs::Float64::Ptr cmd_l(new std_msgs::Float64);
		std_msgs::Float64::Ptr cmd_r(new std_msgs::Float64);
		cmd_l->data = cmd_vx - cmd_yaw;
		cmd_r->data = -(cmd_vx + cmd_yaw);
		pub_l_wheel.publish(cmd_l);
		pub_r_wheel.publish(cmd_r);

		ros::spinOnce();

		loop.sleep();
	}

	return 0;
}

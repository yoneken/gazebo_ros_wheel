#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <boost/thread.hpp>

static int CONTROL_HZ = 50;
static double TRACK_WIDTH = 0.22;
static double WHEEL_RADIUS = 0.05;

static ros::Time last_time;
static double vx_= 0.0, vy_= 0.0, omega_= 0.0, l_omega = 0.0;
static double x_= 0.0, y_= 0.0, theta_= 0.0;
static double duty_left = 0.0, duty_right = 0.0;
static double rad_left = 0.0, rad_right = 0.0, l_rad_left = 0.0, l_rad_right = 0.0;
static double cmd_vx = 0.0, cmd_yaw = 0.0;

static const int joint_num = 2;
static const std::string joint_names[joint_num] = {"wheel_left_joint", "wheel_right_joint"};

std::string odom_frame_name, base_frame_name;
tf::TransformBroadcaster *odom_broadcaster;
nav_msgs::Odometry odom;
ros::Publisher pub_odom;

boost::mutex mtx;

void cmd_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	cmd_vx = msg->linear.x;
	cmd_yaw = msg->angular.z;
}

void js_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
	// search joint_name from message
	int indices[joint_num];
	for(int i=0;i<joint_num;i++){
		for(int j=0;j<joint_num;j++){
			if(joint_names[i] == msg->name[j]){
				indices[i] = j;
				break;
			}
		}
	}

	boost::mutex::scoped_lock lock(mtx);
	rad_left = msg->position[indices[0]];
	rad_right = msg->position[indices[1]];
}

void update_odometry(void)
{
	double d_rad_left, d_rad_right;
	d_rad_left = rad_left - l_rad_left;
	d_rad_right = rad_right - l_rad_right;

	if(d_rad_left != 0.0 && d_rad_right != 0.0){
		double v, vl, vr, l_omega;
		double dx_, dy_, dtheta_;
		vl = WHEEL_RADIUS * -d_rad_left;
		vr = WHEEL_RADIUS * d_rad_right;

		omega_ = (vr - vl) / TRACK_WIDTH;
		dtheta_ = omega_ - l_omega;

		v = (vr + vl) / 2.;
		dx_ = v * cos(dtheta_);
		dy_ = v * sin(dtheta_);

		x_ += dx_;
		y_ += dy_;
		theta_ += dtheta_;

		boost::mutex::scoped_lock lock(mtx);
		l_rad_left = rad_left;
		l_rad_right = rad_right;
		l_omega = omega_;
	}
}

void publish_odom_tf(const ros::Time current_time)
{
	boost::mutex::scoped_lock lock(mtx);
	tf::Quaternion q;
	q.setRPY(0, 0, theta_);
	tf::Transform transform(q, tf::Vector3(x_, y_, 0.0) );
	//transform.setOrigin( tf::Vector3(x_, y_, 0.0) );
	//transform.setRotation(q);
	odom_broadcaster->sendTransform(tf::StampedTransform(transform, current_time, base_frame_name, odom_frame_name));
}

void publish_odom_topic(const ros::Time current_time, const double dt)
{
	odom.header.stamp = current_time;
	odom.header.frame_id = odom_frame_name;
	odom.child_frame_id = base_frame_name;

	boost::mutex::scoped_lock lock(mtx);

	odom.twist.twist.linear.x = (x_ - odom.pose.pose.position.x)/dt;
	odom.twist.twist.linear.y = (y_ - odom.pose.pose.position.y)/dt;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.z = (theta_ - odom.twist.twist.angular.z)/dt;

	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;
	odom.pose.pose.position.z = 0.0;

	geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta_);
	odom.pose.pose.orientation = q;

	pub_odom.publish(odom);
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rrcar_controller");
	ros::NodeHandle nh;

	// load parameters
	std::string odom_topic_name, cmd_topic_name;
	std::string l_wheel_topic_name, r_wheel_topic_name, joint_states_topic_name;
	nh.param<std::string>("odom_topic", odom_topic_name, "odom");
	nh.param<std::string>("cmd_topic", cmd_topic_name, "cmd_vel");
	nh.param<std::string>("l_wheel_topic", l_wheel_topic_name, "wheel_left/effort_controller/command");
	nh.param<std::string>("r_wheel_topic", r_wheel_topic_name, "wheel_right/effort_controller/command");
	nh.param<std::string>("joint_states_topic", joint_states_topic_name, "joint_states");

	nh.param<std::string>("odom_frame_name", odom_frame_name, "odom");
	nh.param<std::string>("base_frame_name", base_frame_name, "base_footprint");

	nh.param("control_hz", CONTROL_HZ, 50);
	nh.param("track_width", TRACK_WIDTH, 0.22);
	nh.param("wheel_radius", WHEEL_RADIUS, 0.05);

	pub_odom = nh.advertise<nav_msgs::Odometry>(odom_topic_name, 10);
	ros::Subscriber sub_cmd = nh.subscribe(cmd_topic_name, 10, cmd_callback);
	ros::Publisher pub_l_wheel = nh.advertise<std_msgs::Float64>(l_wheel_topic_name, 10);
	ros::Publisher pub_r_wheel = nh.advertise<std_msgs::Float64>(r_wheel_topic_name, 10);
	ros::Subscriber sub_js = nh.subscribe(joint_states_topic_name, 10, js_callback);

	tf::TransformBroadcaster br;
	odom_broadcaster = &br;

	last_time = ros::Time::now();
	ros::Rate loop(CONTROL_HZ);

	while(ros::ok()){
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		update_odometry();
		publish_odom_tf(current_time);
		publish_odom_topic(current_time, dt);

		// command each motor
		std_msgs::Float64::Ptr cmd_l(new std_msgs::Float64);
		std_msgs::Float64::Ptr cmd_r(new std_msgs::Float64);
		cmd_l->data = cmd_vx - cmd_yaw;
		cmd_r->data = -(cmd_vx + cmd_yaw);
		pub_l_wheel.publish(cmd_l);
		pub_r_wheel.publish(cmd_r);

		last_time = current_time;

		ros::spinOnce();

		loop.sleep();
	}

	return 0;
}

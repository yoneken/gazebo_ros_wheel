#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <controller_manager/controller_manager.h>
#include <rrcar_control/TwoWheeled.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rrcar_control");
	ros::NodeHandle nh;

	rrcar_control::TwoWheeled rrcar(nh);
	controller_manager::ControllerManager cm(&rrcar, nh);

	ros::Rate rate(1.0 / rrcar.getPeriod()->data.toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time last_time = rrcar.getTime()->data;

	while(ros::ok())
	{
		//ros::Time now = rrcar.getTime()->data;
		//ros::Duration dt = now - last_time;
		std_msgs::TimeConstPtr now = rrcar.getTime();
		std_msgs::DurationPtr dt(new std_msgs::Duration);
		dt->data = now->data - last_time;

		rrcar.read(now, dt);
		cm.update(now->data, dt->data);

		rrcar.write(now, dt);

		last_time = now->data;
		rate.sleep();
	}
	spinner.stop();

	return 0;
}

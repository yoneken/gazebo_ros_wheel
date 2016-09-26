#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <rrcar_control/TwoWheeled.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "base_control");
	ros::NodeHandle nh;

	TwoWheeled base(nh);
	controller_manager::ControllerManager cm(&base, nh);

	ros::Rate rate(1.0 / base.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok())
	{
		ros::Time now = base.getTime();
		ros::Duration dt = base.getPeriod();

		base.read(now, dt);
		cm.update(now, dt);

		base.write(now, dt);
		rate.sleep();
	}
	spinner.stop();

	return 0;
}

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <rrcar_control/TwoWheeled.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rrcar_control");
	ros::NodeHandle nh;

	rrcar_control::TwoWheeled rrcar(nh);
	controller_manager::ControllerManager cm(&rrcar, nh);

	ros::Rate rate(1.0 / rrcar.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok())
	{
		ros::Time now = rrcar.getTime();
		ros::Duration dt = rrcar.getPeriod();

		rrcar.read(now, dt);
		cm.update(now, dt);

		rrcar.write(now, dt);
		rate.sleep();
	}
	spinner.stop();

	return 0;
}

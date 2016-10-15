#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <controller_manager/controller_manager.h>
#include <rrcar_control/TwoWheeled.h>
#include <nodelet/nodelet.h>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_list_macros.h>

namespace rrcar_control
{

class RRCarDriverNodelet: public nodelet::Nodelet
{
public:

	RRCarDriverNodelet() {}

protected:
	ros::Timer timer_;
	ros::Time last_time_;

	virtual void onInit(void);
	void timer_Callback(const ros::TimerEvent& event);

	boost::shared_ptr<TwoWheeled> rrcar;
	boost::shared_ptr<controller_manager::ControllerManager> cm;

}; // class RRCarDriverNodelet

void RRCarDriverNodelet::onInit()
{
	NODELET_DEBUG("Initializing RRCar driver nodelet");
	ros::NodeHandle nh = getNodeHandle();

	rrcar.reset(new TwoWheeled(nh));
	cm.reset(new controller_manager::ControllerManager(rrcar.get(), nh));
	timer_ = nh.createTimer(rrcar->getPeriod()->data, &RRCarDriverNodelet::timer_Callback, this);

	last_time_ = rrcar->getTime()->data;
}

void RRCarDriverNodelet::timer_Callback(const ros::TimerEvent& event)
{
	std_msgs::TimeConstPtr now = rrcar->getTime();
	std_msgs::DurationPtr dt(new std_msgs::Duration);
	dt->data = now->data - last_time_;

	rrcar->read(now, dt);
	cm->update(now->data, dt->data);

	rrcar->write(now, dt);

	last_time_ = now->data;
}

} // namespace rrcar_control

PLUGINLIB_EXPORT_CLASS(rrcar_control::RRCarDriverNodelet, nodelet::Nodelet);

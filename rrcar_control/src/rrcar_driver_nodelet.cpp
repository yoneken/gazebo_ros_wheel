#include <ros/ros.h>
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
	timer_ = nh.createTimer(rrcar->getPeriod(), &RRCarDriverNodelet::timer_Callback, this);
}

void RRCarDriverNodelet::timer_Callback(const ros::TimerEvent& event)
{
	ros::Time now = rrcar->getTime();
	ros::Duration dt = rrcar->getPeriod();

	rrcar->read(now, dt);
	cm->update(now, dt);

	rrcar->write(now, dt);
}

} // namespace rrcar_control

PLUGINLIB_EXPORT_CLASS(rrcar_control::RRCarDriverNodelet, nodelet::Nodelet);

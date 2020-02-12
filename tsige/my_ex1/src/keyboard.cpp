#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

void cmdRCCallback(const geometry_msgs::Twist::ConstPtr& cmd_RC){
	float m_straight=0;
	float m_turn=0;
	m_straight=abs(240*cmd_RC->linear.x);
	m_turn=cmd_RC->angular.z;
	ROS_INFO("%f",cmd_RC->linear.x);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tele");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmdRCCallback);
	ros::Rate r(10);

	while (ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

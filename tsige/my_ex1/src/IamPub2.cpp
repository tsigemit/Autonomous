#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "IamPub2");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message2", 1000);
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "I am node2";
		msg.data = ss.str();
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

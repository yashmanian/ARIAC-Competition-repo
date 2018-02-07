#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	// Initialise node
	ros::init(argc, argv, "pub_test");

	// Node handle
	ros::NodeHandle nh;

	// Publisher
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	// Seed random number generator
	srand(time(0));

	// Loop at 10Hz until shutdown
	ros::Rate rate(10);

	// Loop

	while(ros::ok())
	{
		geometry_msgs::Twist msg;
		msg.linear.x = double(rand())/double(RAND_MAX);
		msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

		// Publish message
		pub.publish(msg);

		// Send message to ROSOUT
		ROS_INFO_STREAM("Random velocity command:"<<"Linear x:"<<msg.linear.x<<"Angular z:"<<msg.angular.z);

		// Delay till next iteration
		rate.sleep();
	}
}
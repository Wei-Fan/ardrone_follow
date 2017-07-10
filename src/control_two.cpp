#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <fstream>

using namespace std;

class ControlTwo
{
public:
	ControlTwo();
private:
	ros::NodeHandle node;
	ros::Subscriber angle_sub;
	ros::Subscriber dist_sub;
	ros::Publisher cmd_pub;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	
	void angleCallback(const std_msgs::Float32 &msg);
	void distantCallback(const std_msgs::Float32 &msg);

	geometry_msgs::Twist cmd;
};

ControlTwo::ControlTwo()
{
	angle_sub = node.subscribe("ardrone_angle", 1, &ControlTwo::angleCallback, this);
	dist_sub = node.subscribe("ardrone_distant", 1, &ControlTwo::distantCallback, this);
	cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
	takeoff_pub = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	land_pub = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
}

void ControlTwo::angleCallback(const std_msgs::Float32 &msg)
{
	//mode transfer

	float w = 0.01*msg.data;

	cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = w;

    cmd_pub.publish(cmd);
}

void ControlTwo::distantCallback(const std_msgs::Float32 &msg)
{
	//mode transfer

	float v = 0.01*(msg.data - 1.0);

	cmd.linear.x = v;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_two");
	ControlTwo t;
	ros::spin();
	return 0;
}
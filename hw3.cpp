//
//
//
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
//create global publisher
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;


//move turtle straight
void move(double speed, double dist);
//rotate turtle
// positive angle = counter clockwise
void rotate (double ang_speed, double angl);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	
	//test case -- change later
	move (1.0, 2.0);
	rotate (2.0, 1.19);
	move (1.0, 2.0);
	rotate (2.0, 2.17);
	move (1.0, 1.6);
	rotate (2.0, 4.7);
	move (1.0, 1.6);
	rotate (2.0, 2.19);
	move (1.0, 2.0);
	
	ros::spin();

}

void move(double speed, double dist)
{
	geometry_msgs::Twist vel_msg;
	double t0 = ros::Time::now().toSec();
	double curr_dist = 0;
	ros::Rate loop_rate(100);
	
	vel_msg.linear.x = abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		curr_dist = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	
	}while (curr_dist < dist);
	
	//force turtle to stop moving once loop is complete
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}

//turtle does not move linearly, simply rotates
void rotate (double ang_speed, double angl)
{
	geometry_msgs::Twist vel_msg;
	double curr_ang = 0.0;	
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);

	//no linear velocity
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = abs(ang_speed);
	
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		curr_ang = ang_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	
	}while (curr_ang < angl);
	
	//force turtle to stop moving once loop is complete
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}

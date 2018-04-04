#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <stdlib.h>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>

using namespace std;

struct TurtlePose {
  std::string turtlename;
  std::string topicname;
  turtlesim::Pose pose;
};

//static ros::ServiceClient sClient;
static ros::ServiceClient kClient;

//create global publisher
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

double getDistance(const double x1, const double y1, const double x2, const double y2);
bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
void removeTurtle(std::string turtlename);
void move(double speed, double dist);
void rotate (double ang_speed, double angl);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);


int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//move( 0.5, 3);
	removeTurtle("/turtle1/");
	ros::spin();

}

// Euclidian distance
double getDistance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt(pow((x1-x2),2) + pow(y1-y2, 2));
}

// Check if contact is close
bool isTooClose(double x1, double y1, double x2, double y2, double threshhold) {
  if (getDistance(x1, y1, x2, y2) <= threshhold)
     return true;
  else
     return false;
}

// Remove captured turtle
void removeTurtle(std::string turtlename) {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = turtlename.c_str();
  if (!kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
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

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
	
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do{
		//linear velocity 
		vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <stdlib.h>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#define NUM_T 7 // Number of T turtles

using namespace std;
int g_iterator = 1; // For tracking vector of pose positions
bool calledback = false; // For tracking successful callback per turtle

//publisher and subscriber
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
//test
ros::Subscriber T_pose_subscriber;
turtlesim::Pose turtlesim_pose;

struct TurtlePose {
  std::string turtlename;
  std::string topicname;
  turtlesim::Pose pose;
};

struct TurtleStruct {
	turtlesim::Pose pose;
	bool isX;
	bool isT;
};

static ros::ServiceClient sClient;
static ros::ServiceClient kClient;

double getDistance(const double x1, const double y1, const double x2, const double y2);
bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
void removeTurtle(std::string turtlename);
void move(double speed, double dist);
void rotate (double ang_speed, double angl);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void addToVector();
//
void poseCB(const turtlesim::Pose::ConstPtr & pose_message);

vector<TurtleStruct> spawnedTurtles;
int vIndex = 0;
float x, y, t;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
		
	for(int i = 1; i < NUM_T + 1; i++){
		string tempturtle = "/T0/pose";
		tempturtle[2] = 48 + i;
		printf("%s\n", tempturtle.c_str());
	
		addToVector();
		//try to subscribe to Tturtles
		T_pose_subscriber = n.subscribe(tempturtle.c_str(), 10, poseCB);
		while(calledback = false){
			ros::spinOnce();
		}
		calledback = false; // Reset
	}
	for(int i = 0; i < spawnedTurtles.size(); i++){
		ROS_INFO("X position of T%d turtle is %f .", i+1, spawnedTurtles[i].pose.x);
		ROS_INFO("Y position of T%d turtle is %f .", i+1, spawnedTurtles[i].pose.y);
		ROS_INFO("THETA position of T%d turtle is %f .", i+1, spawnedTurtles[i].pose.theta);
	}
	
	/*
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	
	addToVector();
	//try to subscribe to Tturtles
	T_pose_subscriber = n.subscribe("/T1/pose", 10, poseCB);
	*/
	
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

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}


void poseCB(const turtlesim::Pose::ConstPtr & pose_message)
{
	spawnedTurtles[g_iterator].pose.x = pose_message -> x;
	spawnedTurtles[g_iterator].pose.y = pose_message -> y;
	spawnedTurtles[g_iterator].pose.theta = pose_message -> theta;
	g_iterator++;
	calledback = true;
	//ROS_INFO("X position of T1 turtle is %f .", spawnedTurtles[0].pose.x);
	//ROS_INFO("Y position of T1 turtle is %f .", spawnedTurtles[0].pose.y);
	//ROS_INFO("THETA position of T1 turtle is %f .", spawnedTurtles[0].pose.theta);
	
	
}

void addToVector()
{
	TurtleStruct temp;
	temp.pose.x = 0;
	temp.pose.y = 0;
	temp.pose.theta = 0;
	
	spawnedTurtles.push_back(temp);
	
	/*
	TurtleStruct temp;
	temp.pose.x = turtlesim_pose.x;
	temp.pose.y = turtlesim_pose.y;
	temp.pose.theta = turtlesim_pose.theta;
	
	if (temp.pose.theta < 2) {
		temp.isT == true;
		temp.isX == false;
	}
	else {
		temp.isT == false;
		temp.isX == true;
	}
	
	spawnedTurtles.push_back(temp);
	*/
}

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
int g_iterator = 0; // For tracking vector of pose positions
bool calledback = false; // For tracking successful callback per turtle
string tempturtle = "/T1/pose"; // First name for searching for T turtles

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
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
void removeTurtle(std::string turtlename);
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
	ROS_INFO("Scanning for friendly turtles...\n");
	
	while(ros::ok() && g_iterator < NUM_T){
		//try to subscribe to Tturtles
		T_pose_subscriber = n.subscribe(tempturtle.c_str(), 10, poseCB);
		ros::spinOnce();
	}
	ROS_INFO("All friendly turtles found! Now moving to capture.");
	T_pose_subscriber.shutdown();
	
	// Start capturing turtles, closest one at a time
	bool found[NUM_T] = {false};
	bool allfound = false;
	while(allfound = false){
		double current_distance[NUM_T];
		for(int i = 0; i < NUM_T; i++){
			current_distance[i] = getDistance(turtlesim_pose.x, turtlesim_pose.y, spawnedTurtles[i].pose.x, spawnedTurtles[i].pose.y);
		}
		int closest = 0;
		for(int i = 0; i < NUM_T - 1; i++){
			if(current_distance[i+1] < current_distance[i] && found[i] == false){
				closest = i + 1;
			}
		}
		moveGoal(spawnedTurtles[closest].pose, 0.5);
		found[closest] = true;
		
		// Check if all turtles found
		allfound = true;
		for(int i = 0; i < NUM_T; i++){
			if(found[i] == false) allfound = false;
		}
	}
	
	ros::spin();
}

// Euclidian distance
double getDistance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt(pow((x1-x2),2) + pow(y1-y2, 2));
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		double Kv = 0.2;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.x = (Kv * e);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		double Kw = 1;
		vel_msg.angular.z = Kw*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	} while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
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

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}

void poseCB(const turtlesim::Pose::ConstPtr & pose_message)
{
	addToVector();
	spawnedTurtles[g_iterator].pose.x = pose_message -> x;
	spawnedTurtles[g_iterator].pose.y = pose_message -> y;
	spawnedTurtles[g_iterator].pose.theta = pose_message -> theta;
	calledback = true;
	ROS_INFO("X position of T%d turtle is %f .", g_iterator + 1, spawnedTurtles[g_iterator].pose.x);
	ROS_INFO("Y position of T%d turtle is %f .\n", g_iterator + 1, spawnedTurtles[g_iterator].pose.y);
	g_iterator++;
	tempturtle[2] = 48 + g_iterator + 1;
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

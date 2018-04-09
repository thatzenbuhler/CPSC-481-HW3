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
#define NUM_X 17 // Number of X turtles

using namespace std;
int g_iterator = 0; // For tracking vector of pose positions
bool calledback = false; // For tracking successful callback per turtle
string tempturtle = "/T1/pose"; // First name for searching for T turtles
string Xtempturtle = "/X1/pose"; // First name for searching for T turtles

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
void removeTurtle(std::string turtlename);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void addToVector();
void poseCB(const turtlesim::Pose::ConstPtr & pose_message);
void sortByDistance();

vector<TurtleStruct> spawnedTurtles;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	kClient = n.serviceClient<turtlesim::Kill>("kill");
	
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ROS_INFO("Scanning for friendly turtles...\n");
	
	while(ros::ok() && g_iterator < NUM_T){
		//try to subscribe to Tturtles
		T_pose_subscriber = n.subscribe(tempturtle.c_str(), 10, poseCB);
		ros::spinOnce();
	}
	ROS_INFO("All friendly turtles found! Now moving to enemy turtles.");
	T_pose_subscriber.shutdown();
	
	ROS_INFO("Scanning for enemy turtles...\n");
	
	while(ros::ok() && g_iterator < NUM_X){
		//try to subscribe to X turtles
		T_pose_subscriber = n.subscribe(Xtempturtle.c_str(), 10, poseCB);
		ros::spinOnce();
	}
	ROS_INFO("All enemy turtles found! Now moving to capture.");
	T_pose_subscriber.shutdown();
	
	// Start capturing turtles
	for(int i = 0; i < NUM_T; i++){
		sortByDistance();
		moveGoal(spawnedTurtles[i].pose, 0.5);
		string temp = "/T0/pose";
		temp[2] = 49 + i;
		removeTurtle(temp);
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

// Remove captured turtle
void removeTurtle(std::string turtlename) {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = turtlename;
  if (!kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = - (pose_message -> x);
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
	if (g_iterator < 7)
	{
		ROS_INFO("X position of T%d turtle is %f .", g_iterator + 1, spawnedTurtles[g_iterator].pose.x);
		ROS_INFO("Y position of T%d turtle is %f .\n", g_iterator + 1, spawnedTurtles[g_iterator].pose.y);
		g_iterator++;
		tempturtle[2] = 48 + g_iterator + 1;
	}
	else if ((g_iterator >= 7) && (g_iterator < 17))
	{
		ROS_INFO("X position of X%d turtle is %f .", g_iterator - 6, spawnedTurtles[g_iterator].pose.x);
		ROS_INFO("Y position of X%d turtle is %f .\n", g_iterator - 6, spawnedTurtles[g_iterator].pose.y);
		g_iterator++;
		if (Xtempturtle[2] != 57)
		{
			Xtempturtle[2] = 48 + g_iterator - 6;
		}
		else
		{
			Xtempturtle = "/X10/pose";
		}
	}
}

void addToVector()
{
	TurtleStruct temp;
	temp.pose.x = 0;
	temp.pose.y = 0;
	temp.pose.theta = 0;
	spawnedTurtles.push_back(temp);
}

void sortByDistance(){
	bool moved = true;
	while(moved == true){
		moved = false;
		for(int i = 0; i < NUM_T - 1; i++){
			double a = getDistance(turtlesim_pose.x, turtlesim_pose.y, spawnedTurtles[i].pose.x, spawnedTurtles[i].pose.y);
			double b = getDistance(turtlesim_pose.x, turtlesim_pose.y, spawnedTurtles[i+1].pose.x, spawnedTurtles[i+1].pose.y);
			if(a > b){
				swap(spawnedTurtles[i], spawnedTurtles[i+1]);
				moved = true;
			}
		}
	}
}


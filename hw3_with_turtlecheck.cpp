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
#define PI 3.14159265359 

using namespace std;
int g_iterator = 0; // For tracking vector of pose positions
bool calledback = false; // turtle subs
string tempturtle = "/T1/pose"; // First name for searching for T turtles
string Xtempturtle = "/X1/pose"; // First name for searching for T turtles

// Publisher and subscriber for turtle1
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
// Subscriber for finding turtles
ros::Subscriber T_pose_subscriber;
turtlesim::Pose turtlesim_pose;

struct TurtleStruct {
	turtlesim::Pose pose;
	int id; // For capturing the turtle after vector is rearranged
	//bool isX;
	//bool isT;
};

static ros::ServiceClient kClient; // For capturing turtles

double getDistance(const double x1, const double y1, const double x2, const double y2);
bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
void removeTurtle(std::string turtlename);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void addToVector();
void poseCB(const turtlesim::Pose::ConstPtr & pose_message);
void rotate (double angl, bool cw);
void move(double dist);
double setDesOr(double dar, bool cw);
void checkTurtle1();

vector<TurtleStruct> spawnedTurtles_T;
vector<TurtleStruct> spawnedTurtles_X;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle");
	ros::NodeHandle n;
	//was 10
	ros::Rate loop_rate(1);
	kClient = n.serviceClient<turtlesim::Kill>("kill");
	bool cw;
	double zero = 0.0;
	
	//initialize publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ROS_INFO("Scanning for friendly turtles...\n");
	
	for(int i = 0; i < NUM_T; i++){
		T_pose_subscriber = n.subscribe(tempturtle.c_str(), 10, poseCB);
		while(calledback == false){
			ros::spinOnce();
		}
		calledback = false;
		T_pose_subscriber.shutdown();
	}
	ROS_INFO("All friendly turtles found! Now moving to enemy turtles.");
	
	for(int i = NUM_T; i < NUM_X; i++){
		T_pose_subscriber = n.subscribe(Xtempturtle.c_str(), 10, poseCB);
		while(calledback == false){
			ros::spinOnce();
		}
		calledback = false;
		T_pose_subscriber.shutdown();
	}
	ROS_INFO("All enemy turtles found! Now moving to capture.");
	
	for (int i = 1; i <= NUM_T; i++)
	{
		double near = 100;
		int index = 0;
		int count = 0;
		
		while (count < (NUM_T - i + 1))
		{
			double dist;
			dist = getDistance(turtlesim_pose.x, turtlesim_pose.y, spawnedTurtles_T[count].pose.x, spawnedTurtles_T[count].pose.y);
			if (dist <= near)
			{
				near = dist;
				index = count;
			}
			++count;
		}
	
		ROS_INFO("The closest turtle is T%i at X:%f   Y:%f \n", (index + i - 1), spawnedTurtles_T[index].pose.x, spawnedTurtles_T[index].pose.y);
	
		float tX, tY;
	
		tX = spawnedTurtles_T[index].pose.x - turtlesim_pose.x;
		tY = spawnedTurtles_T[index].pose.y - turtlesim_pose.y;
	
		double angl = fabs(atan2(tY, tX));
		ROS_INFO("The angle to rotate is %f", angl * 180 / PI);
	
		if (tY <= 0)
			cw = true;
		else
			cw = false;
	
		setDesOr(angl, cw);
		move (near);
		
		if(isTooClose(spawnedTurtles_T[index].pose.x, spawnedTurtles_T[index].pose.y, turtlesim_pose.x, turtlesim_pose.y, 0.5)){
			string killT = "T0";
			killT[1] = 49 + (spawnedTurtles_T[index].id);
			removeTurtle(killT.c_str());
		}
		else
			i--;
		
		setDesOr(-angl, -cw); // reset angle to 0
		
		//take T turtle out of vector
		spawnedTurtles_T.erase(spawnedTurtles_T.begin() + index);
	
		ROS_INFO("The location of turtle1 is X:%f   Y:%f   T:%f \n", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);
		
		checkTurtle1(); // Check if Turtle1 is stil alive
	}
	ros::spin();
}

double setDesOr(double dar, bool cw)
{
	double rar = dar - turtlesim_pose.theta;
	ROS_INFO("The relative angle is %f", rar * 180 / PI);
	if (cw)
		ROS_INFO("turtle will move clockwise\n");
	else
		ROS_INFO("turtle will move counter-clockwise\n");
	//cw = ((rar < 0)? true:false);
	rotate(abs(rar), cw);
}

void rotate (double angl, bool cw)
{
	geometry_msgs::Twist vel_msg;
	double curr_ang = 0.0;	
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	double ang_speed = 1;

	//no linear velocity
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = ang_speed;
	
	if(cw)
	{
		vel_msg.angular.z = -abs(ang_speed);
	}
	else
	{
		vel_msg.angular.z = abs(ang_speed);
	}

	
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

void move(double dist)
{
	geometry_msgs::Twist vel_msg;
	double t0 = ros::Time::now().toSec();
	double curr_dist = 0;
	ros::Rate loop_rate(100);
	double speed = 1;
	
	vel_msg.linear.x = speed;
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
void removeTurtle(string turtlename) {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = turtlename;
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
	
	if (g_iterator < 7)
	{
		
		spawnedTurtles_T[g_iterator].pose.x = pose_message -> x;
		spawnedTurtles_T[g_iterator].pose.y = pose_message -> y;
		spawnedTurtles_T[g_iterator].pose.theta = pose_message -> theta;
		spawnedTurtles_T[g_iterator].id = g_iterator;
		calledback = true;
		
		ROS_INFO("X position of T%d turtle is %f .", g_iterator + 1, spawnedTurtles_T[g_iterator].pose.x);
		ROS_INFO("Y position of T%d turtle is %f .\n", g_iterator + 1, spawnedTurtles_T[g_iterator].pose.y);
		g_iterator++;
		tempturtle[2] = 48 + g_iterator + 1;
	}
	else if ((g_iterator >= 7) && (g_iterator < 17))
	{
		
		spawnedTurtles_X[g_iterator - 7].pose.x = pose_message -> x;
		spawnedTurtles_X[g_iterator - 7].pose.y = pose_message -> y;
		spawnedTurtles_X[g_iterator - 7].pose.theta = pose_message -> theta;
		spawnedTurtles_X[g_iterator - 7].id = g_iterator;
		calledback = true;
		
		ROS_INFO("X position of X%d turtle is %f .", g_iterator - 6, spawnedTurtles_X[g_iterator - 7].pose.x);
		ROS_INFO("Y position of X%d turtle is %f .\n", g_iterator - 6, spawnedTurtles_X[g_iterator - 7].pose.y);
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
	
	if (g_iterator < 7)
	{
		spawnedTurtles_T.push_back(temp);
	}
	else if ((g_iterator >= 7) && (g_iterator < 17))
	{
		spawnedTurtles_X.push_back(temp);
	}	
}

void checkTurtle1(){
	bool found = false;
	ros::master::V_TopicInfo alltopics;
	//get all topic names
	ros::master::getTopics(alltopics);

	for (int i=0; i<alltopics.size(); i++) {
		if(alltopics[i].name == "/turtle1/pose")
			found = true;
	};
	if (found)
		return;
	else{
		ROS_INFO("Turtle 1 Destroyed! Mission Failed.");
		// Print total distance / time etc
		ros::shutdown();
		exit(1);
	}
}


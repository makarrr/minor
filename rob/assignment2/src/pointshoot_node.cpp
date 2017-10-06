#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Subscriber goal_subscriber;
geometry_msgs::PoseStamped goal;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point position;


const double PI = 3.14159265359;

void move(double speed, double distance); //move the robot
double getDistance(double x1, double y1, double x2, double y2); //returns distance to the goal
void rotate(double speed, double angle, bool clockwise); //rotate the robot at specified angle
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg); // shows info about position and orentation of the robot
double yaw;
double goalOrientation;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  yaw = tf::getYaw(msg->pose.pose.orientation);
  position = msg->pose.pose.position;
  ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
  ROS_INFO("Orientation->z: [%f], w: [%f]",yaw, msg->pose.pose.orientation.w);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double angle;
  goal.pose.position.x = msg->pose.position.x;
  goal.pose.position.y = msg->pose.position.y;
  goalOrientation = tf::getYaw(msg->pose.orientation);
  angle = atan2(msg->pose.position.y, msg->pose.position.x)-atan2(position.y, position.x);


  double relative_angle_radians = angle - yaw;
  bool clockwise = relative_angle_radians < 0 ? true : false;
  rotate(0.05, fabs(relative_angle_radians), clockwise);
  
  move(0.1, getDistance(position.x, position.y, msg->pose.position.x, msg->pose.position.y));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointshoot_node");
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	pose_subscriber = n.subscribe("odom",1000,positionCallback);
	goal_subscriber = n.subscribe("goal",1000,goalCallback);
	ros::spin();
	return 0;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2)); //formula of the distance in 2d
}

void rotate (double speed, double angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   	vel_msg.angular.z =-abs(speed);
	   else
	   	vel_msg.angular.z =abs(speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
	   }while(current_angle<angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

void move(double speed, double distance){
	geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis
   vel_msg.linear.x =abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(1000);
   do{
	   velocity_publisher.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   velocity_publisher.publish(vel_msg);
}

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <sstream>

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Subscriber goal_subscriber;


double yaw;
geometry_msgs::PoseWithCovariance_< std::allocator< void > >::_pose_type pose;

double getDistance(double x1, double y1, double x2, double y2);
void moveToGoal(const geometry_msgs::PoseStamped::ConstPtr& goal_pose, double distance_tolerance);	//this will move robot to goal by servoing  (control laws)
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg);
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
  ROS_INFO("Orientation->z: [%f]",msg->pose.pose.orientation.z);
  
  yaw = tf::getYaw(msg->pose.pose.orientation);
  pose = msg->pose.pose;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	moveToGoal(msg,0.1);
}

void moveToGoal(const geometry_msgs::PoseStamped::ConstPtr& goal_pose, double distance_tolerance){
	//here is used feedback control (servoing) method, which was described in the slides.
	//error=desired-measured; start point(x,y), goal point(x1,y1)
	//linear velocity v = const* ((x'-x)^2 + (y'-y)^2)^0.5
	//euclidian distance((x'-x)^2 + (y'-y)^2)^0.5 
	//steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(1000);
	do{ //start of feedback loop to make error
		//linear velocity 
		vel_msg.linear.x = 0.5*getDistance(pose.position.x, pose.position.y, goal_pose->pose.position.x, goal_pose->pose.position.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 2*(atan2(goal_pose->pose.position.y - pose.position.y, goal_pose->pose.position.x - pose.position.x)-tf::getYaw(pose.orientation));

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(pose.position.x, pose.position.y, goal_pose->pose.position.x, goal_pose->pose.position.y)>distance_tolerance); //end of feedback loop to make error

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoing_node");
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	pose_subscriber = n.subscribe("odom",1000,positionCallback);
	goal_subscriber = n.subscribe("goal",1000,goalCallback);
	ros::spin();
	return 0;
}


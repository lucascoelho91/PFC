#include <voronoi/Robot.h>

void Robot::publishSpeed(){
	speedPub.publish(this->speed);
}

void Robot::setSpeedPublisher(ros::NodeHandle& nh, std::string topicName){
	speedPub = nh.advertise<geometry_msgs::Twist>(topicName, 10);
}

void Robot::setPoseSubscriber(ros::NodeHandle& nh, std::string topicName){
	poseSub = nh.subscribe(topicName, 2, &Robot::poseCallback, this);
}

void Robot::poseCallback(const nav_msgs::OdometryConstPtr& msg){
	this->poseOdom = *msg;
	pose.x = poseOdom.pose.pose.position.x;
	pose.y = poseOdom.pose.pose.position.y;
}

void Robot::setSpeed(double v, double w){
	speed.linear.x = v;
	speed.angular.z = w;
}

void Robot::setWeight(double p){
	weight = p;
}

double Robot::getPower(){
	return weight;
}

std::string Robot::getName(){
	return name;
}

Robot::Robot(int id, double weight, 
	   		 rgb color,
	    	 std::string name){
	this->id = id;
	this->weight = weight;
	this->color = color;
	this->name = name;
	this->status = IDLE;
}

double Robot::getX(){
	return this->pose.x;
}

double Robot::getY(){
	return this->pose.y;
}

double Robot::getErrorX(){
	return centroid.x - getX();
}	

double Robot::getErrorY(){
	return centroid.y - getY();
}

double Robot::getTheta(){
	return tf::getYaw(poseOdom.pose.pose.orientation);
}

void Robot::clearControlLaw() //clear all fields
{ 
	controlIntegral.x = 0; 
	controlIntegral.y = 0;
	centroid.x = 0;
	centroid.y = 0;
	sum_coord.x=0;
	sum_coord.y=0;
	sum_cost=0;
}

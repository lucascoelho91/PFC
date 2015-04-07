#include <voronoi/robot.h>

void robot::publishSpeed(){
	speedPub.publish(this->speed);
}

void robot::setSpeedPublisher(ros::NodeHandle& nh, std::string topicName){
	speedPub = nh.advertise<geometry_msgs::Twist>(topicName, 10);
}

void setPoseSubscriber(std::string topicName)
	poseSub = nh.advertise<nav_msgs::Odometry>(topicName, 10); 
}

void poseCallback(nav_msgs::OdometryConstPtr& msg){
	this->poseOdom = *msg;
	pose.x = poseOdom.pose.pose.position.x;
	pose.y = poseOdom.pose.pose.position.y;
}

void robot::setSpeed(double v, double w){
	speed.linear.x = v;
	speed.angular.z = w;
}

void robot::setWeight(double p){
	weight = p;
}

void robot::setOcNode(node* n){
	oc_node = n;
}

double robot::getPower(){
	return weight;
}

node* robot::getOcNode(){
	return oc_node;
}

std::string robot::getName(){
	return name;
}

robot::robot(int id, double weight, 
	   		 rgb color;
	    	 std::string name){
	this->id = id;
	this->weight = weight;
	this->color = color;
	this->name = name;
	this->claw = controlLaw();
}


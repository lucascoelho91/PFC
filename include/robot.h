#include <iostream>
#include <fstream>
#include <vector>
#include <queue>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "graphBuilder.h"


typedef class node node;

class controlLaw
{
	public:
	    Vector2 controlIntegral; //Control Integral
	    Vector2 centroid;        //Centroid Position

	    double costFunction;

	    double kp;    //Control gain

	    void clear() //clear all fields
	    { 
	    	controlIntegral.x = 0; 
	    	controlIntegral.y = 0;
	    	centroid.x = 0;
	    	centroid.y = 0;
	    }
};


class robot
{
    protected:
        int id;       //agent ID
        float weight;     //agent weight
        rgb color;        //agent color in visualization
        std::string rname; //name of the agent on the topic
        node* oc_node; 	  //pointer to the node that the agent is occuping
        char status;	  //status
        controlLaw claw;  //control law structure

        geometry_msgs::Twist speed; //speed ROS structure
		nav_msgs::Odometry poseOdom;    //position of the agent
		Vector2 pose;       //simpler way to represent the position

		ros::Publisher speedPub;    //ROS speed publisher
		ros::Subscriber poseSub; //ROS position subscriber

	public:
		void publishSpeed(ros::NodeHandle* n, std::string nodeName= rname + "/" + "cmd_vel", int bufferSize=10);
		void setSpeedPublisher(std::string topicName);
		void setPoseSubscriber(std::string topicName);
		void setSpeed(double v, double w);
		void setWeight(double p);
		void setOcNode(node* n);
		double getPower();
		node* getOcNode();
		std::string getName();
		robot(int id=0, double weight=0, 
			   rgb color,
			    std::string name = "robot");
};


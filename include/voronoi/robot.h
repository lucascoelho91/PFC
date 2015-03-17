
#ifndef ROBOT_H_
#define ROBOT_H_

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

#include <voronoi/rgb.h>
#include <voronoi/Vector2.h>
#include <voronoi/controlLaw.h>


class node;


class robot
{
    public:
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

		void publishSpeed();
		void setSpeedPublisher(ros::NodeHandle& nh, std::string topicName);
		void setPoseSubscriber(ros::NodeHandle& nh, std::string topicName);
		void setSpeed(double v, double w);
		void setWeight(double p);
		void setOcNode(node* n);
		double getPower();
		node* getOcNode();
		std::string getName();
		robot(int id, double weight, 
			   rgb color,
			    std::string name);
};

#endif /* ROBOT_H_ */
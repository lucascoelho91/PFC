
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


class Robot : public controlLaw
{
    public:
        int id;       //agent ID
        float weight;     //agent weight
        rgb color;        //agent color in visualization
        std::string name; //name of the agent on the topic
        char status;	  //status

        geometry_msgs::Twist speed; //speed ROS structure
		nav_msgs::Odometry poseOdom;    //position of the agent
		Vector2 pose;       //simpler way to represent the position

		ros::Publisher speedPub;    //ROS speed publisher
		ros::Subscriber poseSub; //ROS position subscriber

		void publishSpeed();
		void setSpeedPublisher(ros::NodeHandle& nh, std::string topicName);
		void setPoseSubscriber(ros::NodeHandle& nh, std::string topicName);
		void poseCallback(const nav_msgs::OdometryConstPtr& msg);
		void setSpeed(double v, double w);
		void setWeight(double p);
		double getPower();
		double getTheta();
		Vector2 getPose();
		double getX();
		double getY();

		double getErrorX(){
			return goal.x - getX();
		}	

		double getErrorY(){
			return goal.y - getY();
		}

		std::string getName();
		Robot(int id, double weight, 
			   rgb color,
			    std::string name);

		void clearControlLaw() //clear all fields
	    { 
	    	controlIntegral.x = 0; 
	    	controlIntegral.y = 0;
	    	centroid.x = 0;
	    	centroid.y = 0;
	    }
};

#endif /* ROBOT_H_ */
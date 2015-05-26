
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
#include <tf/tf.h>

#include <voronoi/rgb.h>
#include <voronoi/Vector2.h>
#include <voronoi/controlLaw.h>


class node;

const char MOVING = 1;
const char GOAL_REACHED = 2;
const char IDLE = 3;

class Robot : public controlLaw
{
    public:
        int id;       //agent ID
        float weight;     //agent weight
        rgb color;        //agent color in visualization
        std::string name; //name of the agent on the topic
        char status;	  //status
        node* occupied_node;

        double sum_cost;
        Vector2 sum_coord;
        int sum_nodes;

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
		double getErrorX();
		double getErrorY();

		std::string getName();
		Robot(int id, double weight, rgb color, std::string name);

		void clearControlLaw(); //clear all fields
};

#endif /* ROBOT_H_ */

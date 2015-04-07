#include "voronoi/robot.h"

#include <iostream>
#include <ros/ros.h>




int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		ROS_INFO("Please give the id of the robot");	
	}

	int id = atoi(argv[1]);

	ros::NodeHandle n;
	ros::init(argc, argv);


	voronoi voronoiAdapting(id, n);
	
	return 0;
}
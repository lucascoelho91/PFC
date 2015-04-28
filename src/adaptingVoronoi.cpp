#include <voronoi/Robot.h>
#include <voronoi/Voronoi.h>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <ros/ros.h>




int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		ROS_INFO("Please give the id of the robot");	
	}

	int id = atoi(argv[1]);

	std::string nodeName = "voronoi_" + boost::lexical_cast<std::string>(id);


	ros::NodeHandle nh;
	ros::init(argc, argv, nodeName);

	Voronoi voronoiAdapting(&nh, id);

	voronoiAdapting.initRobots();

	voronoiAdapting.runIteration();
	
	return 0;
}
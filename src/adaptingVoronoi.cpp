#include <voronoi/Robot.h>
#include <voronoi/Voronoi.h>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <string>
#include <ros/ros.h>

using namespace std;


int main(int argc,char** argv)
{

	int id = 1;
	int max_iterations = 10;
	const string nodeName = "voronoi_" + boost::lexical_cast<std::string>(id);

	ros::init(argc, argv, "voronoi");
	ros::NodeHandle nh;

	ros::param::get("voronoi/id", id);

	Voronoi voronoiAdapting(&nh, id);

	ros::Duration loopFreq(0.5);

	ros::Timer timer = nh.createTimer(loopFreq, &Voronoi::runIteration, &voronoiAdapting);
	
	sleep(2);

	ros::spin();

	return 0;
}

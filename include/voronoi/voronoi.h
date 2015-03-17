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

#include <voronoi/robot.h>
#include <voronoi/graphBuilder.h>
#include <voronoi/reportGen.h>

class voronoi
{
	private:
		std::vector <robot> robots;
		graph Graph;

		int iterations;

		char* mapDir;
		char* robotsDir;
		char* imagesDir;
		char* resultsDir;

		robot* master;

        ros::NodeHandle& nh;

	public:
		void initRobots();
		void voronoiDijkstra();

		void setROSPublishers(std::string sulfixPose = "odom");
		void setROSSubscribers(std::string sulfixSpeed = "cmd_vel");

        robot* getrobotByID(int id);

		void getParameters();
		voronoi(ros::NodeHandle& n, int id);

        void runIteration();

		reportGen report;
};

class dijkCost  //used as a cell in the priority queue used in dijstra's algorithm
{
    public:
        node* vNode;       // node
        double powerDist;  // (distance - weight)^2 
        robot* rbx;        // pointer to the robot that owns the cell
        double geoDist;    // geodesic distance between the robot and the cell
        node* s;           // neighbor of the robot that initialized that cell

        void clear(){
        	vNode = NULL;
        	powerDist = 0;
        	rbx = NULL;
        	geoDist = 0;
        	s = NULL;
        }

        dijkCost(){
        	clear();
        }
};

class compareCost    //função usada pela Priotity Queue como critério de ordenação
{
    public:
        bool operator() (dijkCost& c1, dijkCost& c2)
        {
            if (c1.powerDist > c2.powerDist)
            {
                return true;
            }
            else return false;
        }
};

double PowerDist(double x, double r)  // calculates the weighted dist
{
    double h;
    h= (x*x - r*r);
    return (h);
}
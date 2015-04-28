#include <iostream>
#include <string>
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

#include <voronoi/Robot.h>
#include <voronoi/graphBuilder.h>
#include <voronoi/reportGen.h>

class Voronoi
{
	private:
		std::vector <Robot> robots;

        std::list <double> Hfunc;
		
        Graph graph;

		int iterations;

		std::string robotsDir;
		std::string imagesDir;
		std::string resultsDir;

		Robot* controlledRobot;

        ros::NodeHandle* nh;

        int controlLawType;

        static const int PIERSON_FIGUEIREDO = 1;
        static const int PIMENTA_FIGUEIREDO = 2;

	public:
		void initRobots();

		void voronoiDijkstra();

        void calcControlLaw();
        void saveCosts();

		void setROSPublishers(std::string sulfixPose = "odom");
		void setROSSubscribers(std::string sulfixSpeed = "cmd_vel");

        Robot* getrobotByID(int id);

		void getParameters();

		Voronoi(ros::NodeHandle* n, int id);

        void runIteration();
};

class dijkCost  //used as a cell in the priority queue used in dijstra's algorithm
{
    public:
        node* vNode;       // node
        double powerDist;  // (distance - weight)^2 
        Robot* rbx;        // pointer to the robot that owns the cell
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


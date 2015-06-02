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
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <voronoi/Robot.h>
#include <voronoi/graphBuilder.h>
#include <voronoi/reportGen.h>

using namespace std;

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

        sensor_msgs::Image imageMsg;
    	ros::Publisher imagePub;

        static const int PIERSON_FIGUEIREDO = 1;
        static const int PIMENTA_FIGUEIREDO = 2;

	public:
		void initRobots();

		void voronoiDijkstra();

        void calcControlLaw();
        void saveCosts();
        void CalculateCentroid();

		void setROSPublishers(std::string sulfixPose = "odom");
		void setROSSubscribers(std::string sulfixSpeed = "cmd_vel");

        Robot* getrobotByID(int id);

		void getParameters();

		Voronoi(ros::NodeHandle* n, int id);

		void weightController();

        void runIteration(const ros::TimerEvent&);
        int getIterations();
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

template <class T>
void debug(const T& myvar, string before, string after);


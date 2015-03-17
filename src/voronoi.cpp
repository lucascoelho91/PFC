#include <voronoi/voronoi.h>


void voronoi::initRobots()  //TODO - USE ROS PARAMETERS!!!
{
	/*int i=0;
	int id;
	rgb color;
	std::string name;

	node* n;

	ifstream robotConfFile;

	robotConfFile.open(robotsDir);

	while(!robotConfFile.eof())
	{
		if(robotConfFile >> id >> weight >> name >> color.red >> color.g >> color.b )
		{
			robot rbx(id, weight, color, name);
			robots[i] = rbx;
		}
		else{
			perror("Error while reading the robot configuration file");
			exit();
		}
		i++;
	}*/
}

void voronoi::voronoiDijkstra()
{
	robot* r;
	node* n, *neighbor;
	dijkCost k;
	double ncost, pdist;

	// priority queue declaration:
	//                    type          container     ordernation method
    std::priority_queue<dijkCost, std::vector<dijkCost>, compareCost > PQ;


    // initializing the robots on the graph and adding them
	for(int i=0; i<robots.size(); i++)
	{
		r = &robots[i];
		n = r->oc_node;

		r->claw.clear();

		k.powerDist = -pow(r->weight,2);
		k.vNode = n;
		k.rbx = r;
		k.geoDist = 0;
		k.s = NULL;

		PQ.push(k);
	}

	// dijkstra loop
	while(!PQ.empty())   // no elements on the queue anymore
	{
		k = PQ.top();    // gets the element on the queue with the lowest weight
		PQ.pop();        // and removes it

		r = k.rbx;
		n = k.vNode;
		

		// if the cost is bigger,there is no reason to continue the calculations
		if (k.powerDist > n->powerDist) continue; 

		if(k.s != NULL) // checks if this is a start node.
		{
			double xdif = k.s->pose.x - r->pose.x;
			double ydif = k.s->pose.y - r->pose.y;

			r->claw.controlIntegral.x += n->phi*(k.geoDist)*xdif;
			r->claw.controlIntegral.y += n->phi*(k.geoDist)*ydif;
		}

		// iterating on all neighbors
		for(int i = 0; i < 8; i++)
		{
			neighbor = n->neighbor[i];

			if(neighbor != NULL) // checks if it is an obstacle!
			{
				ncost = Graph.getSquareSize()*Graph.getSizeMetersPixel();

				if (i%2) 
					ncost *= sqrt(2);   // if it is on diagonal, multiply by sqrt(2)
				pdist = PowerDist(k.geoDist + ncost, r->weight);

				if( neighbor->powerDist > pdist)  // if the cost is lower than the current cost
				{
					neighbor->powerDist = pdist;
					k.powerDist = pdist;
					k.vNode = neighbor;
					k.rbx = r;
					k.geoDist = k.geoDist + ncost;
				}
				if(k.s = NULL)  // a null k.s occurs when it is an start node
					k.s = neighbor;
				PQ.push(k);
			}
		}	
	}
}

robot* voronoi::getrobotByID(int id)
{
	for(std::vector<robot>::iterator it = robots.begin();
		it != robots.end(); ++it)
	{
		if(it->id == id)
			return &(*it);
	}
	return NULL;
}

void voronoi::setROSSubscribers(std::string sulfixPose)
{
	std::vector<robot>::iterator it;
	std::string topic;
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/" + it->getName() + "/" + sulfixPose;
		it->setPoseSubscriber(nh, topic);
	}
}

void voronoi::setROSPublishers(std::string sulfixSpeed)
{
	std::vector<robot>::iterator it;
	std::string topic;
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/" + it->getName() + "/" + sulfixSpeed;
		it->setSpeedPublisher(nh, topic);
	}
}

void voronoi::runIteration()
{
	this->voronoiDijkstra();
	//this->calcControlLaw();   TODO
	//this->publishControlLaw();   TODO
	//this->saveCosts();   TODO
}

voronoi::voronoi(ros::NodeHandle& nh, int id)
{
	int squareSize, sizeMetersPixel;
	std::string mapFile;
	std::string robotsConfFile;

	this->nh = n;

	if(ros::param::get("/voronoi/squareSize", squareSize));
	else{
		ROS_INFO("Error getting parameter square size parameter");
		exit(1);
	}
	if(ros::param::get("/voronoi/sizeMetersPixel", squareSize));
	else{
		ROS_INFO("Error getting parameter: sizeMetersPixel");
		exit(1);
	}
	if(ros::param::get("/voronoi/map",mapFile));
	else{
		ROS_INFO("Error getting parameter: map file.");
		exit(1);
	}
	if(ros::param::get("/voronoi/robotsConfFile",robotsConfFile));
	else{
		ROS_INFO("Error getting parameter: robots configuration file.");
		exit(1);
	}
	grafo.init(mapFile, squareSize, sizeMetersPixel, 240);
	initRobots(robotsConfFile);

	master = getrobotByID(id);


	setROSSubscribers();
	setROSPublishers();
}

int main(int argc, char* argv[])
{
	if(argc != 4)
	{
		ROS_INFO("Invalid number of arguments. Please give the id and name of the robot");	
	}

	int id = atoi(argv[1]);
	std::string name(argv[2]);
	std::string mapFile(argv[3]);

	std::string nodeName = "robot" + name;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;
	voronoi voronoiTesselation(id);


}
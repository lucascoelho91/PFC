#include <voronoi/Voronoi.h>

double PowerDist(double x, double r)  // calculates the weighted dist
{
    double h;
    h= (x*x - r*r);
    return (h);
}

Vector2 vectorSub(Vector2 a, Vector2 b)
{
    Vector2 ans;
    ans.x = a.x - b.x;
    ans.y = a.y - b.y;
    return ans;
}

double vectorScalarProduct(Vector2 a, Vector2 b)
{
    double ans;
    ans = a.x*b.x;
    ans += a.y*b.y;
    return ans;
}

void Voronoi::initRobots()
{
	std::string robotConfFileName;

	if(ros::param::get("/graph/robotConfFileName", robotConfFileName));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	int i=0;
	int id;
	rgb color;
	std::string name;
	node* n;
	double weight;
	std::ifstream robotConfFile;
	robotConfFile.open(robotConfFileName.c_str());

	while(!robotConfFile.eof())
	{
		if(robotConfFile >> id >> weight >> name >> color.r >> color.g >> color.b )
		{
			Robot rbx(id, weight, color, name);
			robots[i] = rbx;
		}
		else{
			perror("Error while reading the robot configuration file");
			exit(1);
		}
		i++;
	}
}


Voronoi::Voronoi(ros::NodeHandle* n, int id_master)
{
	this->nh = n;
	std::string sulfixSpeed, sulfixPose;

	if(ros::param::get("/graph/sulfixSpeed", sulfixSpeed));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/graph/sulfixPose", sulfixPose));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	graph.GetParametersAndBuildGraph();
	this->initRobots();

	controlledRobot = getrobotByID(id_master);

	setROSSubscribers(sulfixPose);
	setROSPublishers(sulfixSpeed);
}




void Voronoi::voronoiDijkstra()
{
	Robot* robot;
	node* n, *neighbor;
	dijkCost k;
	double ncost, pdist;

	// priority queue declaration:
	//                    type          container     ordernation method
    std::priority_queue<dijkCost, std::vector<dijkCost>, compareCost > PQ;


    // initializing the robots on the graph and adding them
	for(int i=0; i<robots.size(); i++)
	{
		robot = &robots[i];
		n = graph.PoseToNode(robot->pose.x, robot->pose.y);

		robot->clearControlLaw();

		k.powerDist = - pow(robot->weight,2);
		k.vNode = n;
		k.rbx = robot;
		k.geoDist = 0;
		k.s = NULL;

		PQ.push(k);
	}

	// dijkstra loop
	while(!PQ.empty())   // no elements on the queue anymore
	{
		k = PQ.top();    // gets the element on the queue with the lowest weight
		PQ.pop();        // and removes it

		robot = k.rbx;
		n = k.vNode;
		

		// if the cost is bigger,there is no reason to continue the calculations
		if (k.powerDist > n->powerDist) continue; 

		if(k.s != NULL) // checks if this is a start node.
		{
			double xdif = k.s->pose.x - robot->pose.x;
			double ydif = k.s->pose.y - robot->pose.y;

			robot->controlIntegral.x += n->phi*(k.geoDist)*xdif;
			robot->controlIntegral.y += n->phi*(k.geoDist)*ydif;
			n->owner = robot;
		}

		// iterating on all neighbors
		for(int i = 0; i < 8; i++)
		{
			neighbor = n->neighbor[i];

			if(neighbor != NULL) // checks if it is an obstacle!
			{
				ncost = graph.getSquareSize()*graph.getSizeMetersPixel();

				if (i%2) 
					ncost *= sqrt(2);   // if it is on diagonal, multiply by sqrt(2)
				pdist = PowerDist(k.geoDist + ncost, robot->weight);

				if( neighbor->powerDist > pdist)  // if the cost is lower than the current cost
				{
					neighbor->powerDist = pdist;
					k.powerDist = pdist;
					k.vNode = neighbor;
					k.rbx = robot;
					k.geoDist = k.geoDist + ncost;
				}
				if(k.s = NULL)  // a null k.s occurs when it is an start node
					k.s = neighbor;
				PQ.push(k);
			}
		}

		//** neighbor best aligned with the goal **//

		node* neighbor;
		Vector2 neighborVector;
		double scalarProduct;
		int maxVec = 0;
		int argMaxIndex = 0;

		for(int i=0; i < robots.size(); i++)
		{
			graph.PoseToNode(robot->pose.x, robot->pose.y);			
			for(int j=0; j< 8; j++)
			{
				neighbor = n->neighbor[i];
				if(neighbor != NULL)
				{
					neighborVector = vectorSub(neighbor->pose, robots[i].pose);
					scalarProduct = vectorScalarProduct(neighborVector, robots[i].controlIntegral);
					if(maxVec < scalarProduct)
					{
						maxVec = scalarProduct;
						argMaxIndex = j;
					}
				}
			}
			robots[i].goal = n->neighbor[argMaxIndex]->pose;
		}
	}
}

Robot* Voronoi::getrobotByID(int id)
{
	for(std::vector<Robot>::iterator it = robots.begin();
		it != robots.end(); ++it)
	{
		if(it->id == id)
			return &(*it);
	}
	return NULL;
}

void Voronoi::setROSSubscribers(std::string sulfixPose)
{
	std::vector<Robot>::iterator it;
	std::string topic;
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/" + it->getName() + "/" + sulfixPose;
		it->setPoseSubscriber(*nh, topic);
	}
}

void Voronoi::setROSPublishers(std::string sulfixSpeed)
{
	std::vector<Robot>::iterator it;
	std::string topic;
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/" + it->getName() + "/" + sulfixSpeed;
		it->setSpeedPublisher(*nh, topic);
	}
}

void Voronoi::runIteration()
{
	this->voronoiDijkstra();
	this->calcControlLaw();
	this->saveCosts();
}

void Voronoi::calcControlLaw()
{
	int i;
	double theta = controlledRobot->getTheta();
	double errorX = controlledRobot->getErrorX();
	double errorY = controlledRobot->getErrorY();
	double kv = controlledRobot->kv;
	double kw = controlledRobot->kw;
	double d = controlledRobot->d;

	double normControlIntegral = controlledRobot->getNormControlIntegral();

	double v = 0, w = 0;

	if(controlLawType == PIERSON_FIGUEIREDO)
	{

	}
	else if(controlLawType == PIMENTA_FIGUEIREDO)
	{
		if(kv > normControlIntegral) // so the robot will eventually stop
		{
			kw = kw/kv * normControlIntegral;
			kv = normControlIntegral;
		}
		double v = kv*(cos(theta)*errorX + sin(theta)*errorY);
        double w = kw*(-sin(theta)*errorX/d + cos(theta)*errorY/d);

        controlledRobot->setSpeed(v, w);
        controlledRobot->publishSpeed();
	}
}

void Voronoi::saveCosts()
{
	double H = 0;
	node* n;
	double dq = graph.getSizeMetersPixel() * graph.getSquareSize();
	for(int i = 0; i < graph.dim.x; i++)
	{
		for(int j = 0; j < graph.dim.y; j++)
		{
			if(graph.getNodeByIndex(i, j)!=NULL)
			{
				if(n->owner!=NULL)
				{
					n = graph.getNodeByIndex(i, j);
					H += (pow(n->powerDist, 2) + pow(n->owner->weight,2))*(n->phi)*dq;
				}
				n->CleanNode();
			}
			
		}
	}
	Hfunc.push_back(H);
}


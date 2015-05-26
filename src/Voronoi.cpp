#include <voronoi/Voronoi.h>

#define DEBUG 1


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

	double kp, kw, d;

	if(ros::param::get("/voronoi/robotConfFileName", robotConfFileName));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/kp", kp));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/kw", kw));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/d", d));
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

    int i = 1;

	int id;
	int red, green, blue;
	rgb color;
	std::string name;
	node* n;
	std::string dirt;
	double weight;
	std::ifstream robotConfFile;
	robotConfFile.open(robotConfFileName.c_str());
	std::getline(robotConfFile, dirt);
	std::getline(robotConfFile, dirt);

	while(!robotConfFile.eof())
	{

		if(robotConfFile >> id >> weight >> red >> green >> blue)
		{

			color.r = (unsigned char) red;
			color.b = (unsigned char) blue;
			color.g = (unsigned char) green;

			Robot rbx(id, weight, color, name);
			rbx.setControllerParameters(kp, kw, d);
			rbx.pose.x = 2*i;
			rbx.pose.y = 2.5*i;
			i++;
			rbx.controlIntegral.x = 0;
			rbx.controlIntegral.y = 0;

			robots.push_back(rbx);
		}
		else{
			perror("Error while reading the robot configuration file");
			break;
		}
	}

}


Voronoi::Voronoi(ros::NodeHandle* n, int id_master)
{
	this->nh = n;
	std::string sulfixSpeed, sulfixPose;

	if(ros::param::get("/voronoi/speedTopic", sulfixSpeed));
    else{
        ROS_INFO("Error getting speed parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/poseTopic", sulfixPose));
    else{
        ROS_INFO("Error getting pose parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/imagesDir", imagesDir));
    else{
        ROS_INFO("Error getting images dir parameter");
        exit(1);
    }


	graph.GetParametersAndBuildGraph();
	graph.ClearGraph();
	this->initRobots();
	iterations = 0;

	controlLawType = PIMENTA_FIGUEIREDO;

	controlledRobot = getrobotByID(id_master);

	setROSSubscribers(sulfixPose);
	setROSPublishers(sulfixSpeed);
}




void Voronoi::voronoiDijkstra()
{
	Robot* robot;
	node* n, *neighbor, *s;
	dijkCost k;
	double ncost, pdist;
	double cost;
	double geodist;

	// priority queue declaration:
	//                    type          container     ordernation method
    std::priority_queue<dijkCost, std::vector<dijkCost>, compareCost > PQ;


    // initializing the robots on the graph and adding them
	for(int i=0; i<robots.size(); i++)
	{
		robot = &robots[i];
		n = graph.PoseToNode(robot->pose.x, robot->pose.y );
		robot->occupied_node = n;
		n->has_robot = true;
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
		s = k.s;
		if(n == NULL) continue;
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


		geodist = k.geoDist;
		// iterating on all neighbors
		for(int i = 0; i < 8; i++)
		{
			neighbor = n->neighbor[i];


			if(neighbor != NULL) // checks if it is an obstacle!
			{
				ncost = graph.getSquareSize()*graph.getSizeMetersPixel();

				if (i%2) 
					ncost *= sqrt(2);   // if it is on diagonal, multiply by sqrt(2)
				pdist = PowerDist(geodist + ncost, robot->weight);

				if( neighbor->powerDist > pdist)  // if the cost is lower than the current cost
				{
					neighbor->powerDist = pdist;
					k.powerDist = pdist;
					k.vNode = neighbor;
					k.rbx = robot;
					k.geoDist = geodist + ncost;
					if(n->has_robot == true)  // a null k.s occurs when it is an start node
						k.s = neighbor;
					else
						k.s = s;
					PQ.push(k);
				}
			}
		}
	}

	//** neighbor best aligned with the goal **//
	Vector2 neighborVector;
	double scalarProduct;
	int maxVec = -9999999;
	int argMaxIndex = -1;

	for(int i=0; i < robots.size(); i++)
	{
		robot = &robots[i];
		n = graph.PoseToNode(robot->pose.x, robot->pose.y);
		maxVec = -9999999;
		argMaxIndex = -1;
		for(int j=0; j< 8; j++)
		{
			neighbor = n->neighbor[j];
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
		if(argMaxIndex==-1)
			robots[i].goal = robots[i].pose;
		else
			robots[i].goal = n->neighbor[argMaxIndex]->pose;
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
	char str[50];
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/robot_";
		sprintf(str, "%d", it->id);
		topic.append(str);
		topic.append("/");
		topic.append(sulfixPose);
		it->setPoseSubscriber(*nh, topic);
	}
}

void Voronoi::setROSPublishers(std::string sulfixSpeed)
{
	std::vector<Robot>::iterator it;
	std::string topic;
	char str[50];
	for(it = robots.begin(); it!=robots.end(); ++it)
	{
		topic = "/robot_";
		sprintf(str, "%d", it->id);
		topic.append(str);
		topic.append("/");
		topic.append(sulfixSpeed);
		it->setSpeedPublisher(*nh, topic);
	}
}

void Voronoi::runIteration(const ros::TimerEvent&)
{
	ros::spinOnce();
	this->voronoiDijkstra();
	this->calcControlLaw();
	this->saveCosts();
	iterations++;
}

int Voronoi::getIterations()
{
	return iterations;
}

void Voronoi::calcControlLaw()
{
	int i;
	Robot* robot;

	for(i=0; i<robots.size(); i++)
	{
		robot = &robots[i];
		double theta = robot->getTheta();
		double errorX = robot->getErrorX();
		double errorY = robot->getErrorY();
		double kv = robot->kv;
		double kw = robot->kw;
		double d = robot->d;

		double normControlIntegral = robot->getNormControlIntegral();

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


			if(fabs(v) > 1)
				v = v/fabs(v);
			if(fabs(w) > 1)
				w = w/fabs(w);

			robot->setSpeed(v, w);

			robot->publishSpeed();
		}
	}
}

void Voronoi::saveCosts()
{
	rgb pixelwhite, pixelgray, pixelblack;
    pixelwhite.r=255;
    pixelwhite.g=255;
    pixelwhite.b=255;

    pixelblack.r=0;
    pixelblack.g=0;
    pixelblack.b=0;

    pixelgray.r=150;
    pixelgray.g=150;
    pixelgray.b=150;

	double H = 0;
	node* n;
	Robot* rbx;
	int x, y;
	double dq = graph.getSizeMetersPixel() * graph.getSquareSize();

    for(int i = 0; i < graph.dim.y; i++)
    {
        for(int j = 0; j < graph.dim.x; j++)
        {
            graph.visualization[i][j]=graph.colors[i][j];
        }
    }


	for(int i = 0; i < graph.vertices.x; i++)
	{
		for(int j = 0; j < graph.vertices.y; j++)
		{
			x = (i * graph.squareSize + graph.squareSize/2);
            y = (j * graph.squareSize + graph.squareSize/2);

			n = graph.getNodeByIndex(i, j);

			if(n!=NULL)
			{

				if(n->owner!=NULL)  // H cost funcition
				{
					H += (pow(n->powerDist, 2) + pow(n->owner->weight,2))*(n->phi)*dq;
					// printing
					graph.FillSquare(x, y, n->owner->color);
				}
				n->CleanNode();

			}
		}
	}
	Hfunc.push_back(H);

	for(int i = 0; i < robots.size(); i++)
    {
        y = robots[i].pose.y/(graph.getSizeMetersPixel());      //atenção nessa linha se voce trocar o sistema de coordenadas
        x = robots[i].pose.x/(graph.getSizeMetersPixel());
        graph.DrawSquare(3, y, x, pixelblack);
    }

	FILE* outfile;
	char str[100];
    sprintf(str, "%s/%d-iteration_%d.ppm", imagesDir.c_str(), controlledRobot->id, iterations);
    outfile=fopen(str, "w+");
    fprintf(outfile, "P6\n");   //P6 SIGNIFICA CODIFICAÇÃO CRU, COLORIDO, ARQUIVO PPM
    fprintf(outfile, "%d %d\n", (int)graph.dim.x, (int)graph.dim.y);
    fprintf(outfile, "255\n");  //ESSE NUMERO REPRESENTA O NUMERO QUE VALE COMO BRANCO
    for(int i = (graph.dim.y - 1); i >= 0; i--)
    {
        for(int j=0; j<graph.dim.x; j++)
        {
            fprintf(outfile, "%c", graph.visualization[i][j].r);
            fprintf(outfile, "%c", graph.visualization[i][j].g);
            fprintf(outfile, "%c", graph.visualization[i][j].b);
        }
    }
    fclose(outfile);

}



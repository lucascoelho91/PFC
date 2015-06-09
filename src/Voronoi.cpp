#include <voronoi/Voronoi.h>

#define DEBUG 1


double PowerDist(double x, double r)  // calculates the weighted dist
{
    double h;
    h= (x*x - 5*r*r);
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

	double kp, kw, d, kwp;

	if(ros::param::get("/voronoi/robotConfFileName", robotConfFileName)){}
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/kp", kp)){}
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/kw", kw)){}
    else{
        ROS_INFO("Error getting robot configuration file name parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/kwp", kwp)){}
	    else{
	        ROS_INFO("Error getting robot configuration file name parameter");
	        exit(1);
	    }

	if(ros::param::get("/voronoi/d", d)){}
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
	double xdev, ydev;
	std::ifstream robotConfFile;
	robotConfFile.open(robotConfFileName.c_str());
	std::getline(robotConfFile, dirt);
	std::getline(robotConfFile, dirt);

	while(!robotConfFile.eof())
	{

		if(robotConfFile >> id >> weight >> red >> green >> blue >> xdev >> ydev)
		{
			color.r = (unsigned char) red;
			color.b = (unsigned char) blue;
			color.g = (unsigned char) green;

			Robot rbx(id, weight, color, name);
			rbx.setControllerParameters(kp, kw, d, kwp);
			rbx.setDevianceParameters(xdev, ydev);
			rbx.clearControlLaw();
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

	if(ros::param::get("/voronoi/speedTopic", sulfixSpeed)){}
    else{
        ROS_INFO("Error getting speed parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/poseTopic", sulfixPose)){}
    else{
        ROS_INFO("Error getting pose parameter");
        exit(1);
    }

	if(ros::param::get("/voronoi/imagesDir", imagesDir)){}
    else{
        ROS_INFO("Error getting images dir parameter");
        exit(1);
    }


	graph.GetParametersAndBuildGraph();
	graph.ClearGraph();
	this->initRobots();
	iterations = 0;

	imagePub = nh->advertise<sensor_msgs::Image>("Tesselation", 1);
	controlLawType = PIMENTA_FIGUEIREDO;

	controlledRobot = getrobotByID(id_master);

	setROSSubscribers(sulfixPose);
	setROSPublishers(sulfixSpeed);

	HfuncOutput = fopen("/home/lucas/catkin_ws/src/voronoi/H.txt", "w");
	RobotWeigthOutput = fopen("/home/lucas/catkin_ws/src/voronoi/weightRobots.txt", "w");
	RobotWeigthFunctionOutput =  fopen("/home/lucas/catkin_ws/src/voronoi/WeightFunction.txt", "w");
	TimeAtBegin = ros::Time::now();
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
		n = graph.PoseToNode(robot->pose.x, robot->pose.y);

		if(n ==  NULL)
		{
			robot->status = ON_NULL_NODE;
			robot->occupied_node->has_robot = true;
			n->owner = robot;
		}
		else
		{
			robot->occupied_node = n;
			n->has_robot = true;
		}

		robot->clearControlLaw();

		k.powerDist = PowerDist(0, robot->weight);
		k.vNode = robot->occupied_node;
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
		if(n == NULL || n < reinterpret_cast<node*>(4095) || n->owner != NULL) continue;
		// if the cost is bigger,there is no reason to continue the calculations
		if (k.powerDist > n->powerDist) continue;

		if(k.s != NULL) // checks if this is a start node.
		{
			double xdif = s->pose.x - robot->pose.x;
			double ydif = s->pose.y - robot->pose.y;

			robot->controlIntegral.x += n->phi*(k.geoDist)*xdif;
			robot->controlIntegral.y += n->phi*(k.geoDist)*ydif;
			n->owner = robot;
		}


		geodist = k.geoDist;
		// iterating on all neighbors
		for(int i = 0; i < 8; i++)
		{
			neighbor = n->neighbor[i];


			if(neighbor != NULL && neighbor > reinterpret_cast<node*>(4095)) // checks if it is an obstacle!
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
		if(robot->status == IDLE || robot->status == GOAL_REACHED || robot->status == SHOULD_UPDATE_GOAL || robot->status == ON_NULL_NODE)
		{
			n = robot->occupied_node;
			maxVec = -9999999;
			argMaxIndex = -1;
			if(n == NULL || n < reinterpret_cast<node*>(4095)) continue;
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
			{
				robots[i].goal = robots[i].pose;
				robot->status = IDLE;
			}
			else
			{
				robots[i].goal = n->neighbor[argMaxIndex]->pose;
				robot->status = MOVING;
				robots[i].poseLastGoalSet = robots[i].goal;
			}
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
	this->weightController();
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
	CalculateCentroid();
	double dq = graph.getSizeMetersPixel() * graph.getSquareSize();
	tf::Quaternion thetaQ;

	for(i=0; i<robots.size(); i++)
	{
		robot = &robots[i];
		if(robot->status == MOVING)
		{
			double theta = robot->getTheta();
			double errorX = robot->getErrorX();
			double errorY = robot->getErrorY();
			double kv = robot->kv;
			double kw = robot->kw;
			double d = robot->d;

			double v = 0, w = 0;
			double thetaError = 0, d0 = 0, norm = 0;
			double normControlIntegral = robot->getNormControlIntegral();

			if(controlLawType == PIERSON_FIGUEIREDO)
			{

			}
			else if(controlLawType == PIMENTA_FIGUEIREDO)
			{
				theta = robot->getTheta();

				norm = robot->getNormError();

				errorX = robot->getXDotDeviance(errorX);
				errorY = robot->getYDotDeviance(errorY);

				if(kv > normControlIntegral) // so the robot will eventually stop
				{
					kw = kw/kv * normControlIntegral;
					kv = normControlIntegral;
				}



				double v = kv*(cos(theta)*errorX + sin(theta)*errorY);
				double w = kw*(-sin(theta)*errorX/d + cos(theta)*errorY/d);

				if( isnan(v) || isnan(w))
				{
					v = 0;
					w = 0;
				}

				if(fabs(v) > 1)
					v = v/fabs(v);
				if(fabs(w) > 3)
					w = 3*w/fabs(w);

				robot->setSpeed(v, w);

				robot->publishSpeed();

				if(norm < graph.getSquareSize()*graph.getSizeMetersPixel()/4)
					robot->status = GOAL_REACHED;
			}
		}
		else if(robot->status == ON_NULL_NODE || robot->status == IDLE || robot->status == SHOULD_UPDATE_GOAL)
			robot->setSpeed(0,0);
			robot->publishSpeed();
	}
}


double wrapAngle(double angle)
{
	angle = fmod(angle, PI);
	if (angle < -PI)
	   angle += 2*PI;
	return angle;
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
					graph.FillSquare(y, x, n->owner->color);
				}
				n->CleanNode();
			}
		}
	}
	Hfunc.push_back(H);

	for(int i = 0; i < robots.size(); i++)
    {
        y = robots[i].goal.y/(graph.getSizeMetersPixel());
        x = robots[i].goal.x/(graph.getSizeMetersPixel());
        graph.DrawArrow(4, 1, y, x, pixelgray);

        y = robots[i].pose.y/(graph.getSizeMetersPixel());      //atenção nessa linha se voce trocar o sistema de coordenadas
        x = robots[i].pose.x/(graph.getSizeMetersPixel());
        graph.DrawSquare(3, y, x, pixelblack);

        graph.DrawCircle(x, y, robots[i].weight*30, pixelblack, 0);
    }

	imageMsg.height=graph.dim.y;
	imageMsg.width=graph.dim.x;
	imageMsg.encoding="rgb8";
	imageMsg.is_bigendian=false;
	imageMsg.step=graph.dim.x*3;
	imageMsg.data.resize(3*graph.dim.x*graph.dim.y);

	char str[100];
	int k=0;
    sprintf(str, "%s/%d-iteration_%d.ppm", imagesDir.c_str(), controlledRobot->id, iterations);
    for(int i = (graph.dim.y - 1); i >= 0; i--)
    {
        for(int j=0; j<graph.dim.x; j++)
        {
            imageMsg.data[k] = graph.visualization[i][j].r; k++;
			imageMsg.data[k] = graph.visualization[i][j].g; k++;
			imageMsg.data[k] = graph.visualization[i][j].b; k++;
        }
    }

    sprintf(str, "%s/%d-iteration_%d.png", imagesDir.c_str(), controlledRobot->id, iterations);
    cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
	}

	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	imagePub.publish(imageMsg);
	ROS_ASSERT( cv::imwrite( str,  cv_ptr->image ) );

	ros::Duration timespan = - (TimeAtBegin - ros::Time::now());
	double timespanDouble = timespan.sec + (double) timespan.nsec/1000000000;
	fprintf(HfuncOutput, "%f %f\n", timespanDouble, Hfunc.back());

	fprintf(RobotWeigthOutput, "\n%f ", timespanDouble);
	fprintf(RobotWeigthFunctionOutput, "\n%f ", timespanDouble);

	for(int i = 0; i < robots.size(); i++)
	{
		fprintf(RobotWeigthOutput, "%f ", robots[i].weight);
		fprintf(RobotWeigthFunctionOutput, "%f ", robots[i].weight - robots[i].getKiNorm());
	}
	fflush(HfuncOutput);
	fflush(RobotWeigthOutput);
	fflush(RobotWeigthFunctionOutput);
}



void Voronoi::CalculateCentroid()
{
    int i, j;
    node*  n;
    Robot* robot;
    double dens, pos;

    for(i=0; i < robots.size(); i++)
    {
    	robot = &robots[i];
    	robot->sum_nodes = 0;
    	robot->sum_coord.x = 0;
    	robot->sum_coord.y = 0;
    	robot->mass = 0;
    }
    for(i=0; i<graph.vertices.y; i++)
    {
        for(j=0; j<graph.vertices.x; j++)
        {
            n = graph.getNodeByIndex(j, i);
            if(n != NULL  && n->owner != NULL)
            {
            	robot = n->owner;
                dens = n->phi;
                robot->mass+= dens;
                robot->sum_coord.x+= (n->pose.x) * dens;
                robot->sum_coord.y+= (n->pose.y) * dens;
                robot->sum_nodes++;
            }
        }
    }

    for(i=0; i < robots.size(); i++)
    {
    	robot = &robots[i];
        pos = robot->sum_coord.x/(robot->mass);
        robot->centroid.x = pos;

        pos = robot->sum_coord.y/(robot->mass);
        robot -> centroid.y= pos;
    }
}

void Voronoi::weightController()
{
	double weight_i, fk_i, weight_j, fk_j;
	Robot *robot_i, *robot_j;
	double kwp, M_i;

	double w_dot;
	for(int i = 0; i < robots.size(); i++)
	{
		robot_i = &robots[i];
		kwp = robot_i->kwp;
		M_i = robot_i->mass;
		w_dot = 0;
		for(int j = 0; j < robots.size(); j++)
		{
			robot_j = &robots[j];
			w_dot += (-kwp/M_i)*( (robot_i->weight - robot_i->getKiNorm()) - (robot_j->weight - robot_j->getKiNorm()));
		}
		robot_i->weight += w_dot;
	}
}




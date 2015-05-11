
//#ifndef NODE_H_
//#define NODE_H_

#include <voronoi/Robot.h>
#include <voronoi/Vector2.h>


class node  //represents the nodees of the graph
{
    public:
        Vector2  pose;     //xyz position
        int8_t occupiable;     //obstacles?
        node* neighbor[8]; //pointers to the neighbors of the cell
	    Robot* owner; //robot that owns this node
        double powerDist; //powerDist calculated at this node
        double phi;  //density function

        void CleanNode()
        {
        	owner = NULL;
        	powerDist = 99999999;
        }
};

//#endif

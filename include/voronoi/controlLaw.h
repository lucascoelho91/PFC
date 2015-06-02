#include <iostream>
#include <math.h>
#include <voronoi/Vector2.h>

class controlLaw
{
	public:
	    Vector2 controlIntegral; //Control Integral
	    Vector2 centroid;        //Centroid Position

	    double kv;    //Control gain for linear velocity
	    double kw;  //Control gain for angular velocity

	    double kwp; // control gain for weight control

	    double d; // distance for the feedback linearization controller

	    double xdelta;
	    double ydelta;

	    Vector2 goal;

	    void setControllerParameters(double kv_p= 1, double kw_p = 1, double d_p = 0.05, double kwp_p = 0.005);
	    double getNormControlIntegral();
	    void clear(); //clear all fields
	    void setDevianceParameters(double p_xdev, double p_ydev);
	    double getXDotDeviance(double xdot);
	    double getYDotDeviance(double ydot);
	    double getKiNorm();
};

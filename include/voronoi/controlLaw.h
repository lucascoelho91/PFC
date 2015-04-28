#include <iostream>
#include <math.h>

class controlLaw
{
	public:
	    Vector2 controlIntegral; //Control Integral
	    Vector2 centroid;        //Centroid Position

	    double kv;    //Control gain
	    double kw;  //Control gain w

	    double d; // distance for the feedback linearization controller

	    Vector2 goal;

	    void clear() //clear all fields
	    { 
	    	controlIntegral.x = 0; 
	    	controlIntegral.y = 0;
	    	centroid.x = 0;
	    	centroid.y = 0;
	    }

	double getNormControlIntegral(){
		return sqrt(pow(controlIntegral.x,2) + pow(controlIntegral.y, 2));
	}	
};
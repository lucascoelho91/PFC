#include <iostream>

class controlLaw
{
	public:
	    Vector2 controlIntegral; //Control Integral
	    Vector2 centroid;        //Centroid Position

	    double kp;    //Control gain

	    void clear() //clear all fields
	    { 
	    	controlIntegral.x = 0; 
	    	controlIntegral.y = 0;
	    	centroid.x = 0;
	    	centroid.y = 0;
	    }
};
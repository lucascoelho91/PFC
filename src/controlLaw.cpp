#include <voronoi/controlLaw.h>

void controlLaw::clear() //clear all fields
{
	controlIntegral.x = 0;
	controlIntegral.y = 0;
	centroid.x = 0;
	centroid.y = 0;
}

void controlLaw::setControllerParameters(double kv_p, double kw_p, double d_p, double kwp_p)
{
	kv = kv_p;
	kw = kw_p;
	d = d_p;
	kwp = kwp_p;
}

double controlLaw::getNormControlIntegral(){
	return sqrt(pow(controlIntegral.x,2) + pow(controlIntegral.y, 2));
}

void controlLaw::setDevianceParameters(double p_xdev, double p_ydev)
{
	xdelta = p_xdev;
	ydelta = p_ydev;
}

double controlLaw::getXDotDeviance(double xdot)
{
	return xdot*xdelta;
}

double controlLaw::getYDotDeviance(double ydot)
{
	return ydot*ydelta;
}


double controlLaw::getKiNorm()
{
	return (xdelta + kv) + (ydelta + kv);
}



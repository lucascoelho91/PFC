#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <stdexcept>
#include <utility> 

class reportGen{
	std::vector< std::pair<double, double> > var;
	std::string fileName;
	std::ofstream file;
	char separator;

	reportGen(std::string n, char s='\n');
	void insert(double first, double second);
	void saveOnFile();
};

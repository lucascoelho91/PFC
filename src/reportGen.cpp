#include <voronoi/reportGen.h>


reportGen::reportGen(std:string n, char s='\n'){
	fileName = n;
	file.open(fileName);
}

void reportGen::insert(T const& elem){
	try{
		var.insert(elem);
	}
	catch(exception const& ex){
		cerr << "Exception" << ex.what();
	}
	
}

void reportGen::saveOnFile(){
	std::vector<T>::iterator it;
	for( it = var.begin(); it != var.end() ; ++it){
		file << it->first << it->second;
	}
}
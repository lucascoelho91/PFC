cmake_minimum_required(VERSION 2.8.3)
project(voronoi)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
	sensor_msgs
	cv_bridge
	image_transport
)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES $(PROJECT_NAME)

)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
) 


add_executable(voronoi src/adaptingVoronoi.cpp src/graphBuilder.cpp src/Robot.cpp src/Voronoi.cpp)


add_library(graphLib
   src/graphBuilder.cpp
)

add_library(robotLib
	src/Robot.cpp
)

add_library(voronoiLib
   src/Voronoi.cpp
)

add_library(controlLawLib
	src/controlLaw.cpp
)

target_link_libraries(voronoi
	robotLib	
	graphLib
	voronoiLib
    controlLawLib
   ${catkin_LIBRARIES}
 )



# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

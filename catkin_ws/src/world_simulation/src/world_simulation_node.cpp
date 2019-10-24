////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       world_simulation_node.cpp
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-23-2019
///
///     @brief      This file includes the world simulation class (in ROS) to run and visualize 
///									AutoSM code using RVIZ
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "world_simulation.h"

using namespace simulation;

int main(int argc, char** argv) {

	ros::init(argc, argv, "world_simulation");
	
	ros::NodeHandle n("~");

	WorldSim world_sim(n);

	// Main Loop of sim
	while(ros::ok()) {
		ROS_WARN("\n----- Sim Loop -----\n");
	}

	ros::spinOnce(); 

	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       world_simulation.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-23-2019
///
///     @brief      This file includes the world simulation class (in ROS) to run and visualize 
///									AutoSM code using RVIZ
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdlib.h>
#include <math.h>
#include <random>
#include <ros/ros.h>

#include "simulation/sensor_models.h"
#include "simulation/vehicle_models.h"
#include "math/transformation.h"

namespace simulation {

class WorldSim {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WorldSim(ros::NodeHandle& pnh);
	~WorldSim();


	ros::NodeHandle nh;

	enum ViewMode { RVIZ = 0, ROSBAG = 1};

	ViewMode view_mode { ViewMode::RVIZ };


};



}



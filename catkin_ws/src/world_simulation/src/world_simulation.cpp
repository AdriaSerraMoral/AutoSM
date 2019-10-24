////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       world_simulation.cpp
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-23-2019
///
///     @brief      This file includes the world simulation class (in ROS) to run and visualize 
///									AutoSM code using RVIZ
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "world_simulation.h"

#include "visualization_msgs/Marker.h"

namespace simulation {

	WorldSim::WorldSim(ros::NodeHandle& pnh) : nh(pnh) {

		// TODO: Initialize things here
		ROS_WARN("\n --- Initialized node --- \n");
	}

	WorldSim::~WorldSim() {
		switch( view_mode ) {
			case ViewMode::RVIZ : {


				// TODO: Shutdown the publishers
				ROS_WARN("\n --- Shutdown Publishers --- \n");

				break;
			}
			case ViewMode::ROSBAG : {

				// TODO: Close the rosbag

				ROS_WARN("\n --- Closed Ros Bags --- \n");

				break;
			}
		}
	}


}

using namespace simulation;

int main(int argc, char** argv) {

	ros::init(argc, argv, "world_simulation");
	
	ros::NodeHandle n("~");

	WorldSim world_sim(n);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("vehicle", 1);

	visualization_msgs::Marker vehicle;
	vehicle.header.frame_id = "map";
	vehicle.header.stamp = ros::Time(0.0);
	vehicle.ns = "myns";
	//vehicle.id = k+current_cone_pos.left.x.size()+current_cone_pos.right.x.size()+1;
	vehicle.type = visualization_msgs::Marker::MESH_RESOURCE;
	vehicle.action = visualization_msgs::Marker::ADD;
	vehicle.mesh_resource = "file:///home/adria/AutoSM/catkin_ws/src/world_simulation/include/models/drones/drone1/Drone_dae.dae";
	vehicle.pose.position.x = 0;
	vehicle.pose.position.y = 0;
	vehicle.pose.position.z = 0;
	vehicle.pose.orientation.x = 0.0;
	vehicle.pose.orientation.y = 0.0;
	vehicle.pose.orientation.z = 0.0;
	vehicle.pose.orientation.w = 1.0;
	vehicle.scale.x = 0.1;
	vehicle.scale.y = 0.5;
	vehicle.scale.z = 0.5;
	vehicle.color.a = 1.0;

	double time {0.0};
	double dt {0.1};

	// Simulation Frequency
  ros::Rate loop_rate(1/dt);

  ros::Duration(5).sleep();

	// Main Loop of sim
	while(ros::ok()) {
		
		ROS_WARN("--- TIME IS:  %f \n", time);

		vehicle.header.stamp = ros::Time(time);

		marker_pub.publish(vehicle);

		time += dt;

		ros::spinOnce(); 
		loop_rate.sleep();
	}

	ROS_WARN("\n --- STOPPED SIMULATION --- \n");
	
	return 0;
}
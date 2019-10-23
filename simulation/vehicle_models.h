////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       vehicle_models.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-21-2019
///
///     @brief      This file includes all the vehicle models to simulate vehicle dynamics 
///									for either Monte Carlo Simulations or Software in the Loop
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"
#include "common/constants.h"
#include "dynamical_system.h"

namespace AutoSM {
namespace simulation {
namespace vehicles {

/**
 *	@brief	RigidBody
 *
 *					Base RigidBody system. Here, the states are:
 *					X = [ quat_BG, w_b_B, pos_b_G, vel_b_G ]^T
 *					
 *					Base RigidBody Inputs are:
 *					u = [ Moments_B, Forces_B ]^T
 */

class RigidBody : public dynamics::DynamicalSystem {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	RigidBody() = default;
	~RigidBody() = default;

	explicit RigidBody( const double& mass, const Eigen::Matrix3d& inertia, const double& drag_damping_coeff, 
		const double time_start = 0.0, 
	const Eigen::Vector3d& pos_init = Eigen::Vector3d::Zero(), const Eigen::Quaterniond& quat_init = Eigen::Quaterniond::Identity(), 
	const Eigen::Vector3d& vel_init = Eigen::Vector3d::Zero(), const Eigen::Vector3d& w_init = Eigen::Vector3d::Zero(), const DynamicLimits& limits = DynamicLimits() );

	explicit RigidBody(	const RigidBodyParams& params, const VehicleStates& state_init = VehicleStates(), const double time_start = 0.0, const DynamicLimits& limits = DynamicLimits() );

	void setLimits( const DynamicLimits& limits );

	// TODO: Make input actually ptr-like with eigen to automatically update
	void updateInputs(const Eigen::Vector3d& moments_B, const Eigen::Vector3d& forces_B);

	/**
	 *	@brief	computeOutput
	 *	
	 *					Function to compute the system output
	 *					In a linear system, y = C * x + D * u.
	 *
	 *					For now, output is full state y = X.
	 *
	 */
	void computeOutput( const double time ) override;

	// ----- STATES ------------------------------------------------
	VehicleStates		vehicle_states;


private:

	/**
	 *	@brief	computeDerivatives
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the derivative of their states
	 *					In a linear system, x_dot = A * x + B * u
	 */
	void computeDerivatives( const double time, const VecXd& x, const VecXd& u, VecXd& x_dot ) override;

	/**
	 *	@brief	saturateState
	 *
	 *					Function that takes the limits in the states (if any) and saturates the output 
	 *					(if necessary)
	 */
	void saturateState();

	/**
	 *	@brief	saturateInput
	 *
	 *					Function that takes the limits in the inputs (if any) and saturates them
	 */
	void saturateInput(Eigen::Vector3d& moments_B, Eigen::Vector3d& forces_B);

	// Inputs
	Eigen::Vector3d forces_B		{	Eigen::Vector3d::Zero() };
	Eigen::Vector3d moments_B   {	Eigen::Vector3d::Zero() };

	// Rigid Body properties
	RigidBodyParams							rigid_body;

	// Limits
	DynamicLimits limits;


};

}
}
}
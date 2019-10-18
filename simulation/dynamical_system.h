////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       dynamical_system.cpp
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-17-2019
///
///     @brief      This file includes the dynamical system class that is used to integrate dynamics forward in time
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"

namespace AutoSM {
namespace simulation {
namespace dynamics {

/**
 *	@brief	DynamicalSystem
 *
 *					Virtual class that is used to integrate forward in time systems that have dynamics
 *					n -> number of states
 *					m -> number of outputs
 *
 *					x -> State
 *					u -> input
 *					y -> output
 *
 *					x_dot -> derivative of state
 */
class DynamicalSystem {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 *	@brief	DynamicalSystem
	 *	
	 *					Constructor(s)
	 */
	DynamicalSystem() = default;
	DynamicalSystem( int n_states, int n_inputs, int n_output, const double time_prev = 0.0 );

	/**
	 *	@brief	~DynamicalSystem
	 *	
	 *					Destructor(s)
	 */
	~DynamicalSystem()= default;


	enum IntegrationScheme { EULER = 0, RK4 = 1} integrationScheme;


	/**
	 *	@brief	computeDerivatives
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the derivative of their states
	 *					In a linear system, x_dot = A * x + B * u
	 */
	virtual void computeDerivatives( const double time, const VecXd& x, const VecXd& u, VecXd& x_dot ) = 0;

	/**
	 *	@brief	computeOutput
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the system output
	 *					In a linear system, y = C * x + D * u 
	 */
	virtual void computeOutput( const double time ) = 0;

	/**
	 *	@brief	integrate
	 *	
	 *					Virtual function to integrate a dynamical system using one of the available schemes
	 */
	virtual void integrate( const double time ) = 0;

	/**
	 *	@brief	setInitialTime
	 *	
	 *					Function to set the initial time if there a long delay between initalization and 
	 *					when model starts integrating
	 */
	void setInitialTime( const double time_init );

protected:

	double computeDt( const double time );

	const int N_STATES {0};
	const int N_OUTPUT {0};
	const int N_INPUTS {0};
	
	double time_prev_ {0.0};

	VecXd x_;
	VecXd u_;
	VecXd y_;

	VecXd k1, k2, k3, k4;
	

}; 


}	// namespace dynamics
}	// namespace simulation
}	// namespace AutoSM
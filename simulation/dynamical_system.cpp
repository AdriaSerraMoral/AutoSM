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

#include "common/constants.h"
#include "simulation/dynamical_system.h"

namespace AutoSM {
namespace simulation {
namespace dynamics {

	DynamicalSystem::DynamicalSystem( int n_states, int n_inputs, int n_output, const double time_prev ) :
		x_(n_states), u_(n_inputs), y_(n_output), 
		k1(n_states), k2(n_states), k3(n_states), 
		k4(n_states), 
		N_STATES(n_states), N_INPUTS(n_inputs), N_OUTPUT(n_output), time_prev_(time_prev) {
			x_.setZero();
			u_.setZero();
			y_.setZero();
		}

	double DynamicalSystem::computeDt( const double time ) {
		double dt = time - time_prev_;
		time_prev_ = time;

		if( dt < constants::MIN_DIVD ){
			return constants::MIN_DIVD;
		}

		return dt;
	}

	void DynamicalSystem::integrate( const double time ) {

		// find which integrating scheme are we using and Integrate
		switch( integrationScheme ) {
			case IntegrationScheme::EULER: {

				// get dt
				double dt = computeDt( time );

				// compute derivative at current time with current states and input
				this->computeDerivatives( time, x_, u_, k1);
					
				// integrate with dt	
				x_ += dt * k1;

				break;
			}

			case IntegrationScheme::RK4: {

				// compute dt
				double dt = computeDt(time);
				double dt_2 = 0.5 * dt;

				// get rk4 derivatives
				this->computeDerivatives( time, 				x_, 						u_, k1 );
				this->computeDerivatives( time + dt_2, 	x_ + dt_2 * k1, u_, k2 );
				this->computeDerivatives( time + dt_2, 	x_ + dt_2 * k2, u_, k3 );
				this->computeDerivatives( time + dt, 		x_ + dt * k3, 	u_, k4 );

				// integrate
				x_ += (dt / 6.0) * ( k1 + 2.0 * k2 + 2.0 * k3 + k4);

				break;
			}
		}

	}

	void DynamicalSystem::setInitialTime( const double time_init ) { 
		time_prev_ = time_init;
	}

}	// namespace dynamics
}	// namespace simulation
}	// namespace AutoSM
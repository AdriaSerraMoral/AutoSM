////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       controllers.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-24-2019
///
///     @brief      This file includes all the vehicle controllers to take a rigid-body
///									to our desired state
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "controllers.h"
#include "math/transformation.h"

namespace AutoSM { 
namespace control {

	/**
	 *	@brief	ControllerBase
	 *	
	 *					Constructor, connects pointers and defines number of inputs
	 */
	ControllerBase::ControllerBase( const int n_control, VehicleStates* statePtr, TrajectoryCommand* trajPtr, 
			ControllerStates* ctrlStatePtr ) : 
		n(n_control), vehicle_states_ptr(statePtr), traj_ptr(trajPtr), ctrl_states_ptr(ctrlStatePtr) {

			// If the pointer for controller state is empty, instantiate
			if( !ctrl_states_ptr ){

				// instantiate pointer
				ctrl_states_ptr = new ControllerStates();
				own_ctrl_state_ptr = true;

			}

			output.resize(n);
			output.setZero();
	}

	/**
	 *	@brief	ControllerBase
	 *	
	 *					Destructor, deletes the controller state pointer if it owns it
	 */
	ControllerBase::~ControllerBase() {

		if( own_ctrl_state_ptr ){
			delete ctrl_states_ptr;
		}

	}


	/**
	 *	@brief	initializeController
	 *
	 *					function to initialize each particular controller
	 */
	void ControllerBase::initializeController( const RigidBodyParams& rigid_body, const ControllerGains& gains, 
																			 const DynamicLimits& limits ) {

		this->rigid_body = rigid_body;
		this->gains = gains;
		this->limits = limits;

	}

	/**
	 *	@brief	SaturatePositive
	 *
	 *					Function to saturate input if greater than max if control signal acts positive to dynamics
	 */
	void saturatePositive ( double& control_input, double& control_max, double& control_min, double& error, int& windup,
                       bool& is_saturated ) {

		if (control_input >= control_max && error >= 0.0) {
        windup        = 0;
        control_input = control_max;
        is_saturated  = true;
    } else if (control_input >= control_max && error < 0.0) {
        windup        = 0;
        control_input = control_max;
        is_saturated  = true;
    } else if (control_input <= control_min && error > 0.0) {
        windup        = 0;
        control_input = control_min;
        is_saturated  = true;
    } else if (control_input <= control_min && error <= 0.0) {
        windup        = 0;
        control_input = control_min;
        is_saturated  = true;
    } else {
        windup = 1;
    }

	}

	/**
	 *	@brief	SaturateNegative
	 *
	 *					Function to saturate input if greater than max if control signal acts opposite to dynamics
	 */
	void saturateNegative ( double& control_input, double& control_max, double& control_min, double& error, int& windup,
                       bool& is_saturated ) {
		if (control_input >= control_max && error <= 0.0) {
        windup        = 0;
        control_input = control_max;
        is_saturated  = true;
    } else if (control_input >= control_max && error > 0.0) {
        windup        = 0;
        control_input = control_max;
        is_saturated  = true;
    } else if (control_input <= control_min && error < 0.0) {
        windup        = 0;
        control_input = control_min;
        is_saturated  = true;
    } else if (control_input <= control_min && error >= 0.0) {
        windup        = 0;
        control_input = control_min;
        is_saturated  = true;
    } else {
        windup = 1;
    }

	}


	/**
	 *	@brief	RBDIC
	 *	
	 *					Constructor, connects pointers and defines number of inputs
	 */
	RBDIC::RBDIC( VehicleStates* statePtr, TrajectoryCommand* trajPtr, 
			ControllerStates* ctrlStatePtr ) :
		ControllerBase(6, statePtr, trajPtr) {}


	/**
	 *	@brief	getControl
	 *
	 *					get Force and Moment required to move to desired trajectory (linear dynamcis)
	 */
	void RBDIC::getControl() {

		Mat3d R_BG = vehicle_states_ptr->quat_BG.toRotationMatrix(); 
		Vec3d alpha_des_B = R_BG * traj_ptr->alpha_b_G;

		Mat3d w_b_B_skew = math::skewMatrix( vehicle_states_ptr->w_b_B );

		// invert angular dynamics
		output.segment<3>(0) = rigid_body.inertia * alpha_des_B + w_b_B_skew * ( rigid_body.inertia * vehicle_states_ptr->w_b_B );

		// invert linear dynamics
		output.segment<3>(3) = rigid_body.mass * (traj_ptr->acc_b_G + rigid_body.drag_damp_coeff * vehicle_states_ptr->vel_b_G);

	}	

	/**
	 *	@brief	MRAC
	 *	
	 *					Constructor, connects pointers and defines number of inputs
	 */
    DroneMRAC::DroneMRAC( const int n_control, VehicleStates* statePtr, TrajectoryCommand* trajPtr,
			ControllerStates* ctrlStatePtr ) :
		ControllerBase(n_control, statePtr, trajPtr) {}


    /**
     *	@brief	controlPosition
     *
     *					get Force required to move to desired trajectory (linear dynamcis)
     */
    void DroneMRAC::controlPosition( ) {

        // Get the position error in the Global frame
        ctrl_states_ptr->e_pos_b_G = traj_ptr->pos_b_G - vehicle_states_ptr->pos_b_G;

        // Get the velocity error in the Global Frame
        ctrl_states_ptr->e_vel_b_G = traj_ptr->vel_b_G - vehicle_states_ptr->vel_b_G;

        // Integrate the position error in each axis
        // Get the required acceleration in each axis
        for(size_t idx=0; idx < 3; idx++) {
            // Integrate position error
            ctrl_states_ptr->int_pos_b_G(idx) += ctrl_states_ptr->e_pos_b_G(idx) * ctrl_states_ptr->dt * ctrl_states_ptr->windup_pos(idx);

            // Integrate velocity error
            ctrl_states_ptr->int_vel_b_G(idx) += ctrl_states_ptr->e_vel_b_G(idx) * ctrl_states_ptr->dt * ctrl_states_ptr->windup_vel(idx);

        }

    }

}
}

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


#pragma once

#include "common/types.h"

namespace AutoSM { 
namespace control {
	
	
	/**
	 *	@brief	ControllerBase
	 *
	 *					Base controller class that is inherited and contains all relevant
	 *					data necessary to implement any particular controller. This way, we
	 *					provide a common interface
	 *
	 */
	class ControllerBase { 
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		
		//--------------- Output -----------------------------------------------------
		VecXd				output;

		/**
		 *	@brief	ControllerBase
		 *	
		 *					Constructor, connects pointers and defines number of inputs
		 */
		ControllerBase( const int n_control, VehicleStates* statePtr, TrajectoryCommand* trajPtr, 
			ControllerStates* ctrlStatePtr = nullptr  );
		~ControllerBase();

		/**
		 *	@brief	controlPosition
		 *
		 *					get Force required to move to desired trajectory (linear dynamcis)
		 */
		virtual void		controlPosition( ) = 0;
		virtual VecXd		controlPosition( TrajectoryCommand*	traj_ptr ) = 0;

		/**
		 *	@brief	controlAttitude
		 *
		 *					get Moment required to move to desired trajectory (angular dynamcis)
		 */
		virtual VecXd		controlAttitude( TrajectoryCommand*	traj_ptr ) = 0;
		virtual void		controlAttitude(  ) = 0;

		/**
		 *	@brief	controlVelocity
		 *
		 *					get Force required to move to desired trajectory (linear dynamcis)
		 */
		virtual void		controlVelocity( ) = 0;
		virtual VecXd		controlVelocity( TrajectoryCommand*	traj_ptr ) = 0;


		/**
		 *	@brief	getControl
		 *
		 *					get Force and Moment required to move to desired trajectory (linear dynamcis)
		 */
		virtual VecXd		getControl(	TrajectoryCommand*	traj_ptr ) = 0;
		virtual void		getControl(	 ) = 0;

		/**
		 *	@brief	initializeController
		 *
		 *					function to initialize each particular controller
		 */
		virtual void initializeController( const RigidBodyParams& rigid_body, const ControllerGains& gains, 
																			 const DynamicLimits& limits ) = 0;

		/**
		 *	@brief	SaturatePositive
		 *
		 *					Function to saturate input if greater than max if control signal acts positive to dynamics
		 */
		static void saturatePositive ( double& control_input, double& control_max, double& control_min, double& error, int& windup,
                         bool& is_saturated );

		/**
		 *	@brief	SaturateNegative
		 *
		 *					Function to saturate input if greater than max if control signal acts opposite to dynamics
		 */
		static void saturateNegative ( double& control_input, double& control_max, double& control_min, double& error, int& windup,
                         bool& is_saturated );

	protected:

		// Rigi Body Model
		RigidBodyParams			rigid_body;

		// pointer to rigid body state
		VehicleStates*			vehicle_states_ptr 		{nullptr};

		// pointer to controller states
		ControllerStates*		ctrl_states_ptr				{nullptr};

		// Input to Controller (From motion Planner) 
		TrajectoryCommand*	traj_ptr	 						{nullptr};


		//---------------- Limits ----------------------------------------------------
		DynamicLimits				limits;


		//---------------- Gains  ----------------------------------------------------
		ControllerGains			gains;

		// Control properties

		// number of control outputs
		const int n {0};

		// to know if we need to delete ptr in destructor
		bool own_ctrl_state_ptr 									{false};

	};

	/**
	 *	@brief	RBDIC
	 *
	 *					Rigid Body Dynamic Inversion Controller (RBDIC)
	 *					Used rigid-body model with "perfect" Dynamic Inversion to get the control
	 *					Outputs.
	 *				
	 *					Used for simulation, move with true dynamics 
	 *					6 output (3 Moment, 3 Force)
	 */	
	class RBDIC : public ControllerBase {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW


		RBDIC( VehicleStates* statePtr, TrajectoryCommand* trajPtr, 
			ControllerStates* ctrlStatePtr = nullptr  );
		~RBDIC() = default;

		/**
		 *	@brief	getControl
		 *
		 *					get Force and Moment required to move to desired trajectory (linear dynamcis)
		 */
		void getControl() override;


	};

	/**
	 *	@brief	MRAC
	 *
	 *					Model Reference Adaptive Controller (MRAC)
	 *					Used a reference model with Dynamic Inversion to get the control
	 *					Inputs.
	 *
	 *					Adaptive Elements can be an Integrator or Network
	 */
    class DroneMRAC : public ControllerBase {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DroneMRAC( const int n_control, VehicleStates* statePtr, TrajectoryCommand* trajPtr,
			ControllerStates* ctrlStatePtr = nullptr  );
        ~DroneMRAC() = default;

        enum AdaptiveScheme { INTEGRATOR = 0, NN = 1 };

         /**
          *	@brief	controlPosition
          *
          *					get Force required to move to desired trajectory (linear dynamcis)
          */
         void controlPosition( ) override;
		// VecXd		controlPosition( TrajectoryCommand*	traj_ptr ) override;

		// /**
		//  *	@brief	controlAttitude
		//  *
		//  *					get Moment required to move to desired trajectory (angular dynamcis)
		//  */
		// VecXd		controlAttitude( TrajectoryCommand*	traj_ptr ) override;
		// void		controlAttitude(  ) override;

		// /**
		//  *	@brief	controlVelocity
		//  *
		//  *					get Force required to move to desired trajectory (linear dynamcis)
		//  */
		// void		controlVelocity( ) override;
		// VecXd		controlVelocity( TrajectoryCommand*	traj_ptr ) override;


		// /**
		//  *	@brief	getControl
		//  *
		//  *					get Force and Moment required to move to desired trajectory (linear dynamcis)
		//  */
		// VecXd		getControl(	TrajectoryCommand*	traj_ptr ) override;
		// void		getControl(	 ) override;

		// /**
		//  *	@brief	initializeController
		//  *
		//  *					function to initialize each particular controller
		//  */
		// void initializeController( const RigidBodyParams& rigid_body, const ControllerGains& gains, 
		// 																	 const DynamicLimits& limits ) override;

        void   getNuAdaptivePos();
        void   getNuAdaptiveVel();
        void   getNuAdaptiveAtt();

    private:

         // Pseudocontrol signals ($\nu$)

         // Inner-Loop (IL) -> Angular Dynamics
         Vec3d          alpha_h_IL          {Vec3d::Zero()};        ///< From Hedging PCH
         Vec3d          alpha_ref_IL        {Vec3d::Zero()};        ///< From Reference Model
         Vec3d          alpha_lc_IL         {Vec3d::Zero()};        ///< From Linear Compensator
         Vec3d          alpha_ad_IL         {Vec3d::Zero()};        ///< From Adaptive Element

         // Outer-Loop (OL) -> Linear Dynamics
         Vec3d          acc_h_OL            {Vec3d::Zero()};        ///< From Hedging PCH
         Vec3d          acc_ref_OL          {Vec3d::Zero()};        ///< From Reference Model
         Vec3d          acc_lc_OL           {Vec3d::Zero()};        ///< From Linear Compensator
         Vec3d          acc_ad_OL           {Vec3d::Zero()};        ///< From Adaptive Element


	};


}	// namespace AutoSM
} // namespace control

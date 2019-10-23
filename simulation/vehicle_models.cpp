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

#include "vehicle_models.h"
#include "math/transformation.h"

namespace AutoSM {
namespace simulation {
namespace vehicles {

RigidBody::RigidBody(const double& mass, const Eigen::Matrix3d& inertia, const double& drag_damping_coeff, 
	const double time_start, const Eigen::Vector3d& pos_init, const Eigen::Quaterniond& quat_init, 
	const Eigen::Vector3d& vel_init, const Eigen::Vector3d& w_init, const DynamicLimits& limits ) : 
	dynamics::DynamicalSystem(13, 6, 13, time_start) {

		// Set initial state (assume the input is feasible so do not check for limits)
		x_(0) = quat_init.w();
		x_.segment<3>(1) = quat_init.vec();
		x_.segment<3>(4) = w_init;
		x_.segment<3>(7) = pos_init; 
		x_.segment<3>(10) = vel_init;

		vehicle_states.quat_BG = quat_init;
		
		vehicle_states.pos_b_G = pos_init;
		vehicle_states.vel_b_G = vel_init;

		vehicle_states.w_b_B = w_init;

		rigid_body.mass = mass;
		if ( mass < constants::MIN_DIVD ){
			rigid_body.mass = 1.0;
		}
		rigid_body.inertia = inertia;
		rigid_body.drag_damp_coeff = drag_damping_coeff;

		rigid_body.weight = mass * constants::GRAVITY_MAG;

		this->limits = limits;

		rigid_body.inertia_inv = inertia.inverse();
		rigid_body.mass_inv = (1.0 / rigid_body.mass);
}

RigidBody::RigidBody( const RigidBodyParams& params, const VehicleStates& state_init, const double time_start, const DynamicLimits& limits ) :
dynamics::DynamicalSystem(13, 6, 13, time_start) {
	this->rigid_body = params;
	this->vehicle_states = state_init;
	this->limits = limits;

	x_(0) = vehicle_states.quat_BG.w();
	x_.segment<3>(1) = vehicle_states.quat_BG.vec();
	x_.segment<3>(4) = vehicle_states.w_b_B;
	x_.segment<3>(7) = vehicle_states.pos_b_G; 
	x_.segment<3>(10) = vehicle_states.vel_b_G;


}

void RigidBody::setLimits( const DynamicLimits& limits ) {
	this->limits = limits;
}

void RigidBody::computeDerivatives( const double time, const VecXd& x, const VecXd& u, VecXd& x_dot ) {

	// Get current state
	const Eigen::Quaterniond quat_BG_idx( x(0), x(1), x(2), x(3) );
	const Eigen::Quaterniond quat_GB_idx = quat_BG_idx.conjugate();
	const Eigen::Ref<const Eigen::Vector3d> w_b_B_idx = x.segment<3>(4);
	const Eigen::Ref<const Eigen::Vector3d> pos_b_G_idx = x.segment<3>(7);
	const Eigen::Ref<const Eigen::Vector3d> vel_b_G_idx = x.segment<3>(10);

	// Get current inputs
	const Eigen::Ref<const Eigen::Vector3d> moments_B_idx = u.segment<3>(0);
	const Eigen::Ref<const Eigen::Vector3d> forces_B_idx = u.segment<3>(3);

	// Compute quaternion dynamics
	Eigen::Matrix<double, 4, 3> Omega = 0.5 * math::getOmegaMatrix( quat_GB_idx );
	Eigen::Matrix<double, 4, 1> quat_GB_dot_wxyz = Omega * w_b_B_idx;

	// Compute Forces in the Global frame
	Eigen::Vector3d forces_G_idx = quat_GB_idx * forces_B_idx;

	// Compute Rigid Body dynamics
	x_dot(0) = quat_GB_dot_wxyz(0);
	x_dot.segment<3>(1) = -quat_GB_dot_wxyz.segment<3>(1);
	x_dot.segment<3>(4) = rigid_body.inertia_inv * ( moments_B_idx - w_b_B_idx.cross( rigid_body.inertia * w_b_B_idx ) );
	x_dot.segment<3>(7) = vel_b_G_idx;
	x_dot.segment<3>(10) = rigid_body.mass_inv * forces_G_idx - rigid_body.drag_damp_coeff * vel_b_G_idx;
	x_dot(12) -= constants::GRAVITY_MAG;
}

void RigidBody::updateInputs(const Eigen::Vector3d& moments_B, const Eigen::Vector3d& forces_B) {
	this->moments_B = moments_B;
	this->forces_B = forces_B;
}

void RigidBody::computeOutput( const double time ) {

	// Saturate input
	saturateInput( moments_B, forces_B );
	u_.segment<3>(0) = moments_B;
	u_.segment<3>(3) = forces_B;

	// integrate states 
	this->integrate(time);

	vehicle_states.quat_BG.w() = x_(0);
	vehicle_states.quat_BG.vec() = x_.segment<3>(1);
	vehicle_states.quat_BG.normalize();

	vehicle_states.w_b_B = x_.segment<3>(4);

	vehicle_states.pos_b_G = x_.segment<3>(7);

	vehicle_states.vel_b_G = x_.segment<3>(10);


	// Compute helper states
	Mat3d Rot_BG = vehicle_states.quat_BG.toRotationMatrix();
	Eigen::Vector3d forces_G = Rot_BG.transpose() * forces_B;
	vehicle_states.acc_b_G = rigid_body.mass_inv * forces_G - rigid_body.drag_damp_coeff * vehicle_states.vel_b_G;
	vehicle_states.alpha_b_B = rigid_body.inertia_inv * ( moments_B - vehicle_states.w_b_B.cross( rigid_body.inertia * vehicle_states.w_b_B ) );

	// saturate states
	saturateState();

	x_.segment<3>(4) = vehicle_states.w_b_B;

	x_.segment<3>(7) = vehicle_states.pos_b_G;

	x_.segment<3>(10) = vehicle_states.vel_b_G;

}

void RigidBody::saturateInput(Eigen::Vector3d& moments_B, Eigen::Vector3d& forces_B) {

	for( size_t idx = 0; idx < 3; idx++ ) {
		// saturate maximum force
		if( !std::isnan( limits.forces_B_MAX(idx) ) && forces_B(idx) >= limits.forces_B_MAX(idx) ) {
			forces_B(idx) = limits.forces_B_MAX(idx);
		}	

		// saturate maximum moment
		if( !std::isnan( limits.moments_B_MAX(idx) ) && moments_B(idx) >= limits.moments_B_MAX(idx) ) {
			moments_B(idx) = limits.moments_B_MAX(idx);
		}

		// saturate minimum force
		if( !std::isnan( limits.forces_B_MIN(idx) ) && forces_B(idx) <= limits.forces_B_MIN(idx) ) {
			forces_B(idx) = limits.forces_B_MIN(idx);
		}

		// saturate minimum moment
		if( !std::isnan( limits.moments_B_MIN(idx) ) && moments_B(idx) <= limits.moments_B_MIN(idx) ) {
			moments_B(idx) = limits.moments_B_MIN(idx);
		}


	}

	
}

void RigidBody::saturateState() {
	for( size_t idx = 0; idx < 3; idx++ ) {

		// saturate w_B
		if( !std::isnan( limits.w_B_MAX(idx) ) && vehicle_states.w_b_B(idx) >= limits.w_B_MAX(idx) ) {
			vehicle_states.w_b_B(idx) = limits.w_B_MAX(idx);
			vehicle_states.alpha_b_B(idx) = 0.0;
		}
		if( !std::isnan( limits.w_B_MIN(idx) ) && vehicle_states.w_b_B(idx) <= limits.w_B_MIN(idx) ) {
			vehicle_states.w_b_B(idx) = limits.w_B_MIN(idx);
			vehicle_states.alpha_b_B(idx) = 0.0;
		}

		// saturate pos
		if( !std::isnan( limits.pos_G_MAX(idx) ) && vehicle_states.pos_b_G(idx) >= limits.pos_G_MAX(idx) ) {
			vehicle_states.pos_b_G(idx) = limits.pos_G_MAX(idx);
			vehicle_states.vel_b_G(idx) = 0.0;
			vehicle_states.acc_b_G(idx) = 0.0;
		}
		if( !std::isnan( limits.pos_G_MIN(idx) ) && vehicle_states.pos_b_G(idx) <= limits.pos_G_MIN(idx) ) {
			vehicle_states.pos_b_G(idx) = limits.pos_G_MIN(idx);
			vehicle_states.vel_b_G(idx) = 0.0;
			vehicle_states.acc_b_G(idx) = 0.0;
		}

		// saturate vel
		if( !std::isnan( limits.vel_G_MAX(idx) ) && vehicle_states.vel_b_G(idx) >= limits.vel_G_MAX(idx) ) {
			vehicle_states.vel_b_G(idx) = limits.vel_G_MAX(idx);
			vehicle_states.acc_b_G(idx) = 0.0;
		}
		if( !std::isnan( limits.vel_G_MIN(idx) ) && vehicle_states.vel_b_G(idx) <= limits.vel_G_MIN(idx) ) {
			vehicle_states.vel_b_G(idx) = limits.vel_G_MIN(idx);
			vehicle_states.acc_b_G(idx) = 0.0;
		}
	}
}


}	
}
}
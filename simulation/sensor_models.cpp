////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       sensor_models.cpp
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-16-2019
///
///     @brief      This file includes all the helper math and transformation functions that 
///									are used throughout the AutoSM code.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sensor_models.h"
#include "math/transformation.h"

namespace AutoSM {
namespace simulation {
namespace sensors {

	Sensor::Sensor( const double frame_rate, const double time_prev, const Iso3d& T_LS ) {
		this->frame_rate = frame_rate;
		this->time_prev = time_prev;
		this->T_LS = T_LS;
	}

	/**
	 *	@brief	IMU
	 *
	 *					The noise vector includes the sensor variance as follows:
	 *					vector_sigmas = [ sigma_gyr, sigma_acc, sigma_bg, sigma_ba ]^T
	 */
	IMU::IMU(const double frame_rate, 
			VehicleStates* vehicleStatsPtr, 
			const double time_start, const std::vector<double>& vector_sigmas, 
			const Iso3d& T_LS, const Mat3d& Sf, const Mat3d& Mb, 
			const Vec3d& bias_gyr_constant, const Vec3d& bias_acc_constant, const double tau_bg, const double tau_ba ) : 
		Sensor::Sensor(frame_rate, time_start, T_LS), dynamics::DynamicalSystem(6, 3, 6, time_start), 
		vehicle_states_ptr(vehicleStatsPtr)
		
	{

		this->vector_sigma = vector_sigmas;

		this->Mb = Mb;
		this->Sf = Sf;
		this->Ca = Mb * Sf;

		this->tau_bg = tau_bg;
		this->tau_ba = tau_ba;

		this->bias_gyr_constant = bias_gyr_constant;
		this->bias_acc_constant = bias_acc_constant;

		// Make noise vector always 12, if our noise vector is size < 12, take what we have and assume 0 noise for the rest
		// Recall: noise vector as follows: v_noise = [ noise_gyr, noise_acc, noise_bg, noise_ba ]^T
		vector_noise.resize(12);
		const size_t n_noise = vector_sigmas.size();

					
		// set noise vector
		for( size_t idx = 0; idx < 12; idx++ ) {
			if(idx < n_noise){
				vector_noise.at(idx) = std::normal_distribution<double>( 0.0, vector_sigmas.at(idx) );	
			}
			else{
				vector_noise.at(idx) = std::normal_distribution<double>( 0.0, 0.0 );
			}
		}

	}

	/**
	 *	@brief	computeDerivatives
	 *	
	 *					States: x = [ bias_gyr_dynamic, bias_acc_dynamic ]^T
	 *					
	 */
	void IMU::computeDerivatives( const double time, const VecXd& x, const VecXd& u, VecXd& x_dot ) {

		// First, we compute the new random noises for the bias random walk
		// Then, bias_dot = -bias / tau + nise
		for( size_t idx = 0; idx < 3; idx++ ) {
			w_bg(idx) = vector_noise[idx+6](random_generator);
			w_ba(idx) = vector_noise[idx+9](random_generator);

			x_dot(idx) = -x(idx) / tau_bg + w_bg(idx);
			x_dot(idx+3) = -x(idx+3) / tau_ba + w_ba(idx);
		}
		
	}

	/**
	 *	@brief	computeOutput
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the system output
	 *					y = [gyr, acc]^T
	 */
	void IMU::computeOutput( const double time ) {

		// First, we get the true acceleration at the sensor frame
		const Mat3d Rot_LG_ = vehicle_states_ptr->quat_BG.toRotationMatrix();
		const Vec3d pos_s_L_G = Rot_LG_.transpose() * T_LS.translation();
		const Vec3d acc_L_G_ = vehicle_states_ptr->acc_b_G;
		const Vec3d w_rb_L_G_ = Rot_LG_.transpose() * vehicle_states_ptr->w_b_B;
		const Vec3d alpha_L_G_ = Rot_LG_.transpose() * vehicle_states_ptr->alpha_b_B;

		Vec3d acc_S_G_perfect = acc_L_G_ + alpha_L_G_.cross(pos_s_L_G) + w_rb_L_G_.cross( w_rb_L_G_.cross(pos_s_L_G) );

		// Substract the effect of gravity
		// Again, for this code and system, we assume Local-Tangent Plane (NED, with gravity pointing Down)
		Vec3d gravity_G( 0.0, 0.0, constants::GRAVITY_MAG);
		acc_S_G_perfect -= gravity_G;

		// Now get unbiased gyro measurement
		Vec3d gyr_S_G_perfect = w_rb_L_G_;

		// Get the imu output, perturbed by misalignment and other sensor effects
		y_.segment<3>(0) = Ca * T_LS.linear().transpose() * Rot_LG_ * gyr_S_G_perfect;
		y_.segment<3>(3) = Ca * T_LS.linear().transpose() * Rot_LG_ * acc_S_G_perfect;

		// Add the biases and noise
		bias_gyr_dynamic = x_.segment<3>(0);
		bias_acc_dynamic = x_.segment<3>(3);

		bias_gyr = bias_gyr_constant + bias_gyr_dynamic;
		bias_acc = bias_acc_constant + bias_acc_dynamic;

		// get noise
		for(size_t idx = 0; idx < 3; idx++){
			w_gyr(idx) = sqrt(frame_rate) * vector_noise[idx](random_generator);
			w_acc(idx) = sqrt(frame_rate) * vector_noise[idx+3](random_generator);
		}

		// get imu output (gyr + acc)
		y_.segment<3>(0) += bias_gyr + w_gyr;
		y_.segment<3>(3) += bias_acc + w_acc;

	}

	/**
	*	@brief	getSensorMeasurement
	*
	*					Just an user interface function that will call integrate and computeOutput
	*					to populate the sensor data member variables (acc and gyr)
	*/
	void IMU::getSensorMeasurement(const double time ) 
	{ 

		// First, we propagate to current time
		this->integrate(time);

		// Now, we compute the output
		computeOutput(time);

		// Now we populate our member variable output or output structure
		gyr = y_.segment<3>(0);
		acc = y_.segment<3>(3);

	}


	/**
	 *	@brief	GPS
	 *
	 *					GPS constructor, GPS is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	GPS::GPS( const double frame_rate, 
		VehicleStates* vehicleStatsPtr, 
		const double time_start, const std::vector<double>& vector_sigmas, const Iso3d& T_LS ) : 
		Sensor::Sensor(frame_rate, time_start, T_LS), 
		vehicle_states_ptr(vehicleStatsPtr)  {

		this->vector_sigma = vector_sigmas;

		// resize nosie vector, to size 6 (3D pos, and 3D vel)
		vector_noise.resize(6);
		const size_t n_noise = vector_sigmas.size();

		for( size_t idx = 0; idx < 6; idx++ ) {
			if( idx < n_noise ){
				vector_noise.at(idx) = std::normal_distribution<double>( 0.0, vector_sigmas.at(idx) );	
			} 
			else {
				vector_noise.at(idx) = std::normal_distribution<double>( 0.0, 0.0 );	
			}
		}

	}

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables (pos and vel)
	 */
	void GPS::getSensorMeasurement(const double time) {

		// Get noise
		for( size_t idx = 0; idx < 3; idx++ ) {
			w_pos(idx) = vector_noise[idx](random_generator);
			w_vel(idx) = vector_noise[idx+3](random_generator);
		}

		// The true position of the gps antenna
		const Mat3d Rot_LG_ = vehicle_states_ptr->quat_BG.toRotationMatrix();
		const Vec3d pos_s_L_G = Rot_LG_.transpose() * T_LS.translation();
		const Vec3d pos_L_G_ = vehicle_states_ptr->pos_b_G;

		pos = pos_L_G_ + pos_s_L_G + w_pos;

		// Get the velocity of the gps antenna
		const Vec3d vel_L_G_ = vehicle_states_ptr->vel_b_G;
		const Vec3d w_rb_L_G_ = Rot_LG_.transpose() * vehicle_states_ptr->w_b_B;
		vel = vel_L_G_ + w_rb_L_G_.cross( pos_s_L_G ) + w_vel;

	}


	/**
	 *	@brief	MotionCapture
	 *
	 *					MotionCapture constructor, MotionCapture is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	MotionCapture::MotionCapture( const double frame_rate,  
		VehicleStates* vehicleStatsPtr, 
		const double time_start, const std::vector<double>& vector_sigmas, 
		const Iso3d& T_LS) : 
		Sensor::Sensor(frame_rate, time_start, T_LS), vehicle_states_ptr(vehicleStatsPtr) {

			this->vector_sigma = vector_sigmas;

			// resize nosie vector, to size 6 (3D att, and 3D pos)
			vector_noise.resize(6);
			const size_t n_noise = vector_sigmas.size();

			for( size_t idx = 0; idx < 6; idx++ ) {
				if( idx < n_noise ){
					vector_noise.at(idx) = std::normal_distribution<double>( 0.0, vector_sigmas.at(idx) );	
				} 
				else {
					vector_noise.at(idx) = std::normal_distribution<double>( 0.0, 0.0 );	
				}
			}

	}

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables (pos and vel)
	 */
	void MotionCapture::getSensorMeasurement(const double time) {

		// Get noise
		for( size_t idx = 0; idx < 3; idx++ ) {
			w_att(idx) = vector_noise[idx](random_generator);
			w_pos(idx) = vector_noise[idx+3](random_generator);
		}

		// The true position of the gps antenna
		const Mat3d Rot_LG_ = vehicle_states_ptr->quat_BG.toRotationMatrix();
		const Vec3d pos_s_L_G = Rot_LG_.transpose() * T_LS.translation();
		const Vec3d pos_L_G_ = vehicle_states_ptr->pos_b_G;

		pos = T_LS.linear().transpose() * Rot_LG_ * ( pos_L_G_ + pos_s_L_G ) + w_pos;

		// Get the velocity of the gps antenna
		Mat3d Rot_SG = T_LS.linear().transpose() * Rot_LG_;
		Mat3d noise_mat = math::skewMatrix(w_att);
		Rot_SG = ( Rot_SG * (Mat3d::Identity() + noise_mat) ).eval();

		att = Quatd(Rot_SG);

	}


	/**
	 *	@brief	Altimeter
	 *
	 *					Altimeter constructor, Altimeter is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	Altimeter::Altimeter(const double frame_rate, 
		VehicleStates* vehicleStatsPtr, 
		const double time_start, const std::vector<double>& vector_sigmas, 
		const Iso3d& T_LS) : Sensor::Sensor(frame_rate, time_start, T_LS), vehicle_states_ptr(vehicleStatsPtr) {

		// resize nosie vector, to size 1 (Altitude)
			vector_noise.resize(1);
			const size_t n_noise = vector_sigmas.size();

			for( size_t idx = 0; idx < 1; idx++ ) {
				if( idx < n_noise ){
					vector_noise.at(idx) = std::normal_distribution<double>( 0.0, vector_sigmas.at(idx) );	
				} 
				else {
					vector_noise.at(idx) = std::normal_distribution<double>( 0.0, 0.0 );	
				}
			}
	}

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables
	 */
	void Altimeter::getSensorMeasurement(const double time) {
		w_alt = vector_noise[0](random_generator);

		const Mat3d Rot_LG_ = vehicle_states_ptr->quat_BG.toRotationMatrix();
		const Vec3d pos_s_L_G = Rot_LG_.transpose() * T_LS.translation();
		const Vec3d pos_L_G_ = vehicle_states_ptr->pos_b_G;

		alt = -( pos_L_G_(2) + pos_s_L_G(2) ) + w_alt;
	}


}	// namespace sensors
}	// namespace simulation
}	// namespace AutoSM
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       sensor_models.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-16-2019
///
///     @brief      This file includes all the sensor models to simulate sensor data for either
///									Monte Carlo Simulations or Software in the Loop
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"
#include "common/constants.h"
#include "dynamical_system.h"

namespace AutoSM {
namespace simulation {
namespace sensors {

class Sensor {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Sensor() = default;
	~Sensor() = default;

	Sensor( const double frame_rate, const double time_prev = 0.0, const Iso3d& T_LS = Iso3d::Identity() );


	virtual void getSensorMeasurement(const double time) = 0;

	// Transformation from Local frame (e.g., Body, NED, ...) to Sensor Frame 
	Iso3d	T_LS { Iso3d::Identity() };

	// Noise vector
	std::vector<double>	vector_sigma;

	// Gaussian noise vector
	std::default_random_engine random_generator;
	std::vector<std::normal_distribution<double>> vector_noise;

	// frame rate
	double frame_rate {0};

	// time last time sensor was updated (used to update internal dynamics)
	double time_prev {0.0};

};


// TODO: Take constructor with a RigidBody pointer to set-up references.
// TODO: Use frame_rate in compute Measurement to only get new data at the given rate
// TODO: If Ref<const ...> doesn't work, use map< ptr, size> v and initiallize to null and set ptr in constructor

/**
 *	@brief	IMU
 *
 *					Sensor model of Inertial Measurement Unit to generate synthetic data.
 *					The IMU is attached to a rigid-body, which has dynamics in the Global Frame. 
 *					
 *					The IMU output is gyr (rate gyro output) and acc (accelerometer output)
 *
 */
class IMU : public Sensor, public dynamics::DynamicalSystem  {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 *	@brief	IMU
	 *
	 *					The noise vector includes the sensor variance as follows:
	 *					vector_sigmas = [ sigma_gyr, sigma_acc, sigma_bg, sigma_ba ]^T
	 */
	explicit IMU( const double frame_rate, 
		VehicleStates* vehicleStatsPtr, 
		const double time_start = 0.0, const std::vector<double>& vector_sigmas = std::vector<double>(), 
		const Iso3d& T_LS = Iso3d::Identity(), const Mat3d& Sf = Mat3d::Identity(), 
		const Mat3d& Mb = Mat3d::Identity(), const Vec3d& bias_gyr_constant = Vec3d::Zero(), const Vec3d& bias_acc_constant = Vec3d::Zero(), 
		const double tau_bg = 50.0, const double tau_ba = 100.0 ); 

	~IMU() = default;

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables
	 */
	void getSensorMeasurement(const double time) override;

	const Vec3d& getGyroNoise() const { return w_gyr; }

	const Vec3d& getAccelNoise() const {return w_acc; }

	// Total bias
	Vec3d bias_gyr { Vec3d::Zero() };
	Vec3d bias_acc { Vec3d::Zero() };

	// Simulated IMU data (output)
	Vec3d acc { Vec3d::Zero() };
	Vec3d gyr { Vec3d::Zero() };

private:

	/**
	 *	@brief	computeDerivatives
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the derivative of their states
	 *					In a linear system, x_dot = A * x + B * u
	 */
	void computeDerivatives( const double time, const VecXd& x, const VecXd& u, VecXd& x_dot ) override;

	/**
	 *	@brief	computeOutput
	 *	
	 *					Virtual function to be overritten by dynamical systems to compute the system output
	 *					In a linear system, y = C * x + D * u 
	 */
	void computeOutput( const double time ) override;

	// IMU modeling matrices

	// Misalignment and scale factor matrix
	Mat3d Ca { Mat3d::Identity() };

	// scale factor matrix
	Mat3d Sf { Mat3d::Identity() };

	// misalignment matrix
	Mat3d Mb { Mat3d::Identity() };

	// Markov's process time constant
	double tau_ba { 100.0 };
	double tau_bg { 50.0 };

	// constant bias
	Vec3d bias_gyr_constant { Vec3d::Zero() };
	Vec3d bias_acc_constant { Vec3d::Zero() };

	// dynamic bias (walking markov's process)
	Vec3d bias_gyr_dynamic  { Vec3d::Zero() };
	Vec3d bias_acc_dynamic  { Vec3d::Zero() };

	// noise for measurement(s)
	Vec3d w_gyr  { Vec3d::Zero() };
	Vec3d w_acc  { Vec3d::Zero() };

	// noise for random walk
	Vec3d w_ba { Vec3d::Zero() };
	Vec3d w_bg { Vec3d::Zero() };

	// reference to the true data needed to get imu simulation
	// TODO: Investigate if this is pointer-like and works vs. const Ref<Vector3d> ?
	VehicleStates*	vehicle_states_ptr { nullptr };

};

/**
 *	@brief	GPS
 *
 *					Sensor model of GPS/GNSS to generate synthetic data.
 *					This is a "dirty" initial simulation that basically computes GLOBAL position and velocity
 *					at the GPS antenna. 
 *
 *					
 *					The GPS output is pos and vel
 *
 */
class GPS : public Sensor {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 *	@brief	GPS
	 *
	 *					GPS constructor, GPS is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	explicit GPS( const double frame_rate,  
		VehicleStates* vehicleStatsPtr, 
		const double time_start = 0.0, const std::vector<double>& vector_sigmas = std::vector<double>(), 
		const Iso3d& T_LS = Iso3d::Identity());

	~GPS() = default;

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables
	 */
	void getSensorMeasurement(const double time) override;

	const Vec3d& getPositionNoise() const { return w_pos; }

	const Vec3d& getVelocityNoise() const {return w_vel; }

	Vec3d pos;
	Vec3d vel;

private:

	// Noise vectors
	Vec3d w_pos { Vec3d::Zero() };
	Vec3d w_vel { Vec3d::Zero() };

	// reference to the true data needed to get synthetic gps data
	VehicleStates* vehicle_states_ptr {nullptr}; 

};

/**
 *	@brief	MotionCapture
 *
 *					Sensor model of MotionCapture (e.g., Vicon) to generate synthetic data.
 *					This is a "dirty" initial simulation that basically computes position and attitude in MotionCapture Frame
 *					(one of the motion capture elements)
 *
 *					
 *					The MotionCapture output is Attitude (Eigen Quaternion from Global to MotionCapture) and Position
 *
 */
class MotionCapture : public Sensor {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 *	@brief	MotionCapture
	 *
	 *					MotionCapture constructor, MotionCapture is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	explicit MotionCapture( const double frame_rate,  
		VehicleStates* vehicleStatsPtr, 
		const double time_start = 0.0, const std::vector<double>& vector_sigmas = std::vector<double>(), 
		const Iso3d& T_LS = Iso3d::Identity());

	~MotionCapture() = default;

	/**
	 *	@brief	getSensorMeasurement
	 *
	 *					Just an user interface function that will call integrate and computeOutput
	 *					to populate the sensor data member variables
	 */
	void getSensorMeasurement(const double time) override;

	const Vec3d& getPositionNoise() const { return w_pos; }

	const Vec3d& getAttitudeNoise() const {return w_att; }

	Vec3d pos;
	Quatd att;

private:

	// Noise vectors
	Vec3d w_pos { Vec3d::Zero() };
	Vec3d w_att { Vec3d::Zero() };

	// reference to the true data needed to get synthetic gps data
	VehicleStates* vehicle_states_ptr {nullptr}; 

};

/**
 *	@brief	Altimeter
 *
 *					Notion of altimeter, to generate synthetic altitude measurements 
 *					Essentially, returns -Down + noise
 */
class Altimeter : public Sensor {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 *	@brief	Altimeter
	 *
	 *					Altimeter constructor, Altimeter is not a dynamical object, it has no dynamics or states.
	 *
	 *					
	 */
	explicit Altimeter( const double frame_rate, 
		VehicleStates* vehicleStatsPtr, 
		const double time_start = 0.0, const std::vector<double>& vector_sigmas = std::vector<double>(), 
		const Iso3d& T_LS = Iso3d::Identity());

	~Altimeter() = default;

	void getSensorMeasurement(const double time) override;

	const double& getAltitudeNoise() const { return w_alt; }

	double alt;

private:
	double w_alt;

	VehicleStates* vehicle_states_ptr {nullptr}; 

};
	

}	// namespace sensors
}	// namespace simulation
}	// namespace AutoSM


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       types.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-16-2019
///
///     @brief      This file includes typedefs, structures, and data types that may be common 
///									throughout the code
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdlib.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <random>
#include <memory>

#include "constants.h"

namespace AutoSM {

// Forward-Declaration of structures used in typedefs
struct FrameState;
struct FeatureState;
struct VehicleStates;

// =====================================================================================================================
//
//																					TYPEDEFS	
//	
// =====================================================================================================================

// Unique Ids, used in Estimation for feature tracking and visual-aided navigation
using FeatureId 	= 	int64_t;
using StateId			= 	int64_t;

// Useful Eigen typedefs

// Vectors
using Vec2d     = Eigen::Vector2d;
using Vec3d     = Eigen::Vector3d;
using Vec4d     = Eigen::Vector4d;
using Vec5d     = Eigen::Matrix<double, 5, 1>;
using Vec6d     = Eigen::Matrix<double, 6, 1>;
using Vec7d     = Eigen::Matrix<double, 7, 1>;
using Vec8d     = Eigen::Matrix<double, 8, 1>;
using Vec9d     = Eigen::Matrix<double, 9, 1>;
using Vec10d     = Eigen::Matrix<double, 10, 1>;
using Vec11d     = Eigen::Matrix<double, 11, 1>;
using Vec12d     = Eigen::Matrix<double, 12, 1>;
using Vec13d     = Eigen::Matrix<double, 13, 1>;
using Vec14d     = Eigen::Matrix<double, 14, 1>;
using Vec15d    = Eigen::Matrix<double, 15, 1>;
using Vec16d    = Eigen::Matrix<double, 16, 1>;
using Vec17d    = Eigen::Matrix<double, 17, 1>;
using Vec18d    = Eigen::Matrix<double, 18, 1>;
using Vec19d     = Eigen::Matrix<double, 19, 1>;
using Vec20d    = Eigen::Matrix<double, 20, 1>;
using Vec21d    = Eigen::Matrix<double, 21, 1>;

// Vector Dynamic
using VecXd			= Eigen::VectorXd;

// Row vectors
using RowVec2d = Eigen::Matrix<double, 1, 2>;
using RowVec3d = Eigen::Matrix<double, 1, 3>;
using RowVec4d = Eigen::Matrix<double, 1, 4>;
using RowVec5d = Eigen::Matrix<double, 1, 5>;
using RowVec6d = Eigen::Matrix<double, 1, 6>;
using RowVec7d = Eigen::Matrix<double, 1, 7>;
using RowVec8d = Eigen::Matrix<double, 1, 8>;
using RowVec9d = Eigen::Matrix<double, 1, 9>;
using RowVec10d = Eigen::Matrix<double, 1, 10>;
using RowVec11d = Eigen::Matrix<double, 1, 11>;
using RowVec12d = Eigen::Matrix<double, 1, 12>;
using RowVec13d = Eigen::Matrix<double, 1, 13>;
using RowVec14d = Eigen::Matrix<double, 1, 14>;
using RowVec15d = Eigen::Matrix<double, 1, 15>;
using RowVec16d = Eigen::Matrix<double, 1, 16>;
using RowVec17d = Eigen::Matrix<double, 1, 17>;
using RowVec18d = Eigen::Matrix<double, 1, 18>;
using RowVec19d = Eigen::Matrix<double, 1, 19>;
using RowVec20d = Eigen::Matrix<double, 1, 20>;
using RowVec21d = Eigen::Matrix<double, 1, 21>;

// Matrix Square
using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat7d = Eigen::Matrix<double, 7, 7>;
using Mat8d = Eigen::Matrix<double, 8, 8>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat10d = Eigen::Matrix<double, 10, 10>;
using Mat11d = Eigen::Matrix<double, 11, 11>;
using Mat12d = Eigen::Matrix<double, 12, 12>;
using Mat13d = Eigen::Matrix<double, 13, 13>;
using Mat14d = Eigen::Matrix<double, 14, 14>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Mat16d = Eigen::Matrix<double, 16, 16>;
using Mat17d = Eigen::Matrix<double, 17, 17>;
using Mat18d = Eigen::Matrix<double, 18, 18>;
using Mat19d = Eigen::Matrix<double, 19, 19>;
using Mat20d = Eigen::Matrix<double, 20, 20>;
using Mat21d = Eigen::Matrix<double, 21, 21>;

// Matrix Dynamic
using MatXd = Eigen::MatrixXd;

// Matrix Other
using Mat2x3d = Eigen::Matrix<double, 2, 3>;
using Mat3x4d = Eigen::Matrix<double, 3, 4>;

// Transforms
using Iso3d = Eigen::Isometry3d;
using Quatd = Eigen::Quaterniond;

// Vector of Eigen
using VectorVec2d 	= 	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;
using VectorVec3d 	= 	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;

using VectorMat3d   = 	std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;

using VectorIso3d		= 	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> >;
using VectorQuatd		= 	std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >;


using MeasurementMap	=	std::map< StateId, Eigen::Vector2d, std::less<StateId>, 
																	Eigen::aligned_allocator<std::pair<const StateId, Eigen::Vector2d>> >;

using FeatureIdSet		=	std::unordered_set< FeatureId >;
using FrameIdSet			= std::set<	StateId >;


// Unique pointers to structs
using FeatureUniquePtr		= std::unique_ptr<FeatureState>;
using FrameUniquePtr			= std::unique_ptr<FrameState>;

using VehicleStateUniquePtr = std::unique_ptr<VehicleStates>;
using VehicleStateSharedPtr = std::shared_ptr<VehicleStates>;




// =====================================================================================================================
//
//																					STRUCTURES 
//	
// =====================================================================================================================

/**
 *	@brief	VehicleStates
 *	
 *					Common states to all vehicle(s) or a rigid body
 */
struct VehicleStates	{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VehicleStates(){}

	// Linear Dynamics ( Expressed in Global Frame )
	Vec3d 						pos_b_G								{	Vec3d::Zero()	};
	Vec3d 						vel_b_G								{	Vec3d::Zero()	};
	Vec3d 						acc_b_G								{	Vec3d::Zero()	};
	double						height_AGL						{	0.0	};


	// Angular Dynamcis ( Expressed in Local or Body Frame )
	Quatd 						quat_BG								{	Quatd::Identity()	};
	Vec3d 						w_b_B 								{	Vec3d::Zero() };
	Vec3d 						alpha_b_B 						{	Vec3d::Zero()	};


	// Covariances
	Vec3d 						cov_pos								{	Vec3d::Zero()	};
	Vec3d 						cov_vel								{	Vec3d::Zero()	};
	Vec3d 						cov_acc								{	Vec3d::Zero()	};
	Vec3d 						cov_att								{	Vec3d::Zero()	};
	Vec3d 						cov_w_B								{	Vec3d::Zero()	};
	Vec3d 						cov_hAGL							{	Vec3d::Zero()	};

};

/**
 *	@brief	DynamicLimits
 *
 *					Limits in Vehicle states or dynamcis
 *
 */
struct DynamicLimits {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vec3d w_B_MAX      		{	Vec3d( NAN, NAN, NAN ) 	};
	Vec3d w_B_MIN      		{	Vec3d( NAN, NAN, NAN ) 	};

	Vec3d pos_G_MAX    		{	Vec3d( NAN, NAN, NAN ) 	};
	Vec3d pos_G_MIN    		{	Vec3d( NAN, NAN, NAN ) 	};

	Vec3d vel_G_MAX    		{	Vec3d( NAN, NAN, NAN ) 	};
	Vec3d vel_G_MIN    		{	Vec3d( NAN, NAN, NAN ) 	};

	Vec3d forces_B_MAX		{	Vec3d( NAN, NAN, NAN ) 	};
	Vec3d forces_B_MIN		{	Vec3d( NAN, NAN, NAN ) 	};

	Vec3d moments_B_MAX		{	Vec3d( NAN, NAN, NAN ) 	};
	Vec3d moments_B_MIN		{	Vec3d( NAN, NAN, NAN ) 	};	

	DynamicLimits(){}
};



/**
 *	@brief	ImuStates
 *
 *					States Common to an IMU sensor
 *
 */
struct ImuStates {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Measurement output
	Vec3d 						fs_imu_IMU						{	Vec3d::Zero()	};
	Vec3d 						w_imu_IMU 						{ Vec3d::Zero()	};

	// Bias Estimate
	Vec3d 						bias_gyr							{	Vec3d::Zero()	};
	Vec3d 						bias_acc							{	Vec3d::Zero()	};

	// Extrinsics (From Local to IMU frame)
	Iso3d 						T_L_IMU								{	Iso3d::Identity()	};

	// covariance
	Vec3d 						cov_bg								{	Vec3d::Zero()	};
	Vec3d 						cov_ba								{	Vec3d::Zero()	};
	Vec3d 						cov_pos_L_IMU					{	Vec3d::Zero()	};
	Vec3d 						cov_att_L_IMU					{	Vec3d::Zero()	};

	// Noise Specs
	Vec3d 						sigma_gyr							{	Vec3d::Zero()	};
	Vec3d 						sigma_acc							{	Vec3d::Zero()	};
	Vec3d 						sigma_bg							{	Vec3d::Zero()	};
	Vec3d 						sigma_ba							{	Vec3d::Zero()	};
};


/**
 *	@brief	CameraStates
 *
 *					States common to a Camera sensor
 */
struct CameraStates {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Intrinsics
	double							fx										{	1.0	};
	double							fy										{	1.0	};
	double							cx										{	0.0	};
	double							cy										{	0.0	};
	double							b 										{	0.0	};

	// Extrinsics
	Iso3d 							T_L_CAM								{ Iso3d::Identity()	};
	double							td 										{	0.0	};


	// covariance
	Vec3d 							cov_pos_L_CAM					{	Vec3d::Zero()	};
	Vec3d 							cov_att_L_CAM					{ Vec3d::Zero() };
	double							cov_td 								{ 0.0 };

	
};

/**
 *	@brief	RigidBodyParams
 *
 *					Parameters constant to all rigid bodies
 */
struct RigidBodyParams {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	double						mass										{ 1.0 };

	Mat3d 						inertia									{	Mat3d::Identity() };

	double 						weight									{	constants::GRAVITY_MAG };

	double 						drag_damp_coeff					{	0.3	};

	Mat3d 						inertia_inv							{ Mat3d::Identity() };

	double 						mass_inv								{ 1.0 };

};

/**
 *	@brief	ControllerStates
 *
 *					Structure containing the main states and data used by all controllers
 */
struct ControllerStates {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Error in states
	Vec3d							e_att_BG								{	Vec3d::Zero()	};
	Vec3d							e_w_b_B	 								{	Vec3d::Zero()	};
	Vec3d							e_pos_b_G								{	Vec3d::Zero()	};
	Vec3d							e_vel_b_G								{	Vec3d::Zero()	};

	double						OmegaR									{	0.0	};

	Eigen::Vector3i		windup_att							{	Eigen::Vector3i::Ones()	};
	Eigen::Vector3i		windup_pos							{	Eigen::Vector3i::Ones()	};
	Eigen::Vector3i		windup_vel							{	Eigen::Vector3i::Ones()	};

	Vec3d							int_att_BG							{	Vec3d::Zero()	};
	Vec3d							int_pos_b_G							{	Vec3d::Zero()	};
	Vec3d							int_vel_b_G							{	Vec3d::Zero()	};

	bool 							dynamicsSaturated				{	false	};
	bool							controlSaturated				{	false	};

};

/**
 *	@brief	ControllerGains
 *
 *					Cassic Gains of PiD-like controller or general matrix
 */
struct ControllerGains {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Proportional
	Vec3d 						Kp											{	Vec3d::Zero() };
	Vec3d 						Rp 											{	Vec3d::Zero() };

	// Derivative
	Vec3d 						Kd 											{ Vec3d::Zero() };
	Vec3d 						Rd 											{	Vec3d::Zero() };

	// Integral
	Vec3d 						Ki 											{	Vec3d::Zero() };
	Vec3d 						Ri 											{	Vec3d::Zero() };

	// General control gain matrix
	MatXd 						Kgain										{ MatXd::Ones(1,1) };

};


/**
 *	@brief	TrajectoryCommand
 *
 *					Trajectory, output of the motion planner or guidance module
 */
struct TrajectoryCommand {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vec3d 						pos_b_G							{	Vec3d::Zero() };
	Vec3d 						vel_b_G							{	Vec3d::Zero() };
	Vec3d 						acc_b_G							{	Vec3d::Zero() };
	Vec3d 						jerk_b_G						{	Vec3d::Zero() };

	Quatd 						att_BG							{	Quatd::Identity() };
	Vec3d 						w_b_G								{	Vec3d::Zero() };
	Vec3d 						alpha_b_G						{	Vec3d::Zero() };
	Vec3d 						jang_b_G						{	Vec3d::Zero() };

};


/**
 *	@brief	FrameState
 *
 *					States common to a frame (Stochastic Clone)
 *
 */
struct FrameState {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Unique Ids
	static StateId next_id;
	StateId 							id 								{ 0 };

	// Pose
	Quatd 								quat_GC						{	Quatd::Identity() };
	Vec3d 								pos_cam_G					{	Vec3d::Zero() };

	// Pose (First time it was created)
	Quatd 								quat_GC_null			{	Quatd::Identity() };
	Vec3d 								pos_cam_G_null		{	Vec3d::Zero() };

	// Features observed by frame
	FeatureIdSet 					feature_set;


	// ------------------ FUNCTIONS _------------------------------

	// Constructors
	FrameState() = default;

	FrameState(const Quatd& quat_GC, const Vec3d& pos_cam_G) : id(next_id++) {
		this->pos_cam_G = this->pos_cam_G_null = pos_cam_G;
		this->quat_GC = this->quat_GC_null = quat_GC;
	}

	// get number of features tracked by frame
	inline int featuresTracking() {
		return (int)(feature_set.size());
	}

};

/**
 *	@brief	FeatureState
 *	
 *					States common to Features for Localization and Mapping
 *
 */
struct FeatureState {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Unique Ids
	static FeatureId 				next_id;
	FeatureId  							id 										{ 0 };

	// Frames that are tracking the feature and observation
	MeasurementMap  				measurement_map;

	// tracking stats
	int 										num_tracked_total 		{ 0 };
	bool										is_initialized				{ false };

	// ------------------ FUNCTIONS _------------------------------

	// Constructors
	FeatureState() = default;

	inline StateId 	firstFrameTracked() {
		return (measurement_map.begin()->first);
	}

	inline StateId	lastFrameTracked() {
		return ((--measurement_map.end())->first);
	}

	inline int framesTracking() {
		return (int)(measurement_map.size());
	}
	
};




} // namespace AutoSM

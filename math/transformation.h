////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       transformation.h
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-16-2019
///
///     @brief      This file includes all the helper math and transformation functions that 
///									are used throughout the AutoSM code.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "common/types.h"

namespace AutoSM { 
namespace math {
	

/**
 *	@brief	eulerToRotation
 *
 *					Takes a sequence of 3-2-1 euler angles and returns the rotation matrix
 *					that transforms a vector from the global to the local frame.
 *					In traditional Aerospace, the rotation sequence is yaw-pitch-roll
 *					and the rotation matrix returned is R_LG.
 */
	template <typename T>
	static Eigen::Matrix<T, 3, 3> eulerToRotation(const Eigen::Matrix<T, 3, 1>& euler_321) {
		// get angles
		T psi = euler_321(0);
		T the = euler_321(1);
		T phi = euler_321(2);

		T cpsi = cos(psi);
		T cthe = cos(the);
		T cphi = cos(phi);
		T spsi = sin(psi);
		T sthe = sin(the);
		T sphi = sin(phi);

		// Get rotation matrix from angles 
		Eigen::Matrix<T, 3, 3> R_yaw = (Eigen::Matrix<T, 3, 3>() << cpsi, spsi, T(0.0), -spsi, cpsi, T(0.0), T(0.0), T(0.0), T(1.0)).finished();
		Eigen::Matrix<T, 3, 3> R_pitch = (Eigen::Matrix<T, 3, 3>() << cthe, T(0.0), -sthe, T(0.0), T(1.0), T(0.0), sthe, T(0.0), cthe).finished();
		Eigen::Matrix<T, 3, 3> R_roll = (Eigen::Matrix<T, 3, 3>() << T(1.0), T(0.0), T(0.0), T(0.0), cphi, sphi, T(0.0), -sphi, cphi).finished();

		return (R_yaw * R_pitch * R_roll);
	}

	template <typename T>
	static Eigen::Matrix<T, 3, 3> eulerToRotation(const T& psi, const T& the, const T& phi ) {

		T cpsi = cos(psi);
		T cthe = cos(the);
		T cphi = cos(phi);
		T spsi = sin(psi);
		T sthe = sin(the);
		T sphi = sin(phi);

		// Get rotation matrix from angles 
		Eigen::Matrix<T, 3, 3> R_yaw = (Eigen::Matrix<T, 3, 3>() << cpsi, spsi, T(0.0), -spsi, cpsi, T(0.0), T(0.0), T(0.0), T(1.0)).finished();
		Eigen::Matrix<T, 3, 3> R_pitch = (Eigen::Matrix<T, 3, 3>() << cthe, T(0.0), -sthe, T(0.0), T(1.0), T(0.0), sthe, T(0.0), cthe).finished();
		Eigen::Matrix<T, 3, 3> R_roll = (Eigen::Matrix<T, 3, 3>() << T(1.0), T(0.0), T(0.0), T(0.0), cphi, sphi, T(0.0), -sphi, cphi).finished();

		return (R_yaw * R_pitch * R_roll);
	}

	/**
	 *	@brief	rotationToEuler
	 *
	 *					Function to get 3-2-1 euler angle sequence from a rotation matrix.
	 *					The rotaiton matrix is assumed to be from global to local (e.g., 
	 *					NED to Body)
	 */
	template <typename T>
	static Eigen::Matrix<T, 3, 1> rotationToEuler( const Eigen::Matrix<T, 3, 3>& R_LG ) {
		T yaw = atan2(R_LG(0, 1), R_LG(0, 0));
		T pitch = -asin(R_LG(0, 2));
		T roll = atan2(R_LG(1, 2), R_LG(2, 2));

		return (Eigen::Matrix<T, 3, 1>(yaw, pitch, roll));
	}

	/**
	 *	@brief 	skewMatrix
	 *
	 *					Returns the skew-symmetric matrix of a vector
	 */
	template <typename T>
	static Eigen::Matrix<T, 3, 3> skewMatrix( const Eigen::Matrix<T, 3, 1>& v) {
		return ( Eigen::Matrix<T, 3, 3>() << T(0.0), -v(2), v(1), v(2), T(0.0), -v(0), -v(1), v(0), T(0.0) ).finished();
	}

	/**
	 *	@brief	skewToVec
	 *
	 *					Given a skew symmetric matrix, get the vector
	 */
	template <typename T>
	static Eigen::Matrix<T, 3, 1> skewToVec( const Eigen::Matrix<T, 3, 3>& m_skew ) {
		Eigen::Matrix<T, 3, 1> v;
		v(0) = T(0.5) * ( -m_skew(1, 2) + m_skew(2, 1) );
		v(1) = T(0.5) * (  m_skew(0, 2) - m_skew(2, 0) );
		v(2) = T(0.5) * ( -m_skew(0, 1) + m_skew(1, 0) );

		return v;
	}

	/**
	 *	@brief	getLeftQuaternion
	 *
	 *					returns the matrix for left-multiplying a hamiltonian quaternion.
	 *					We use hamiltonian by default in default templated functions since it is
	 *					what most libraries (Eigen, Ceres, ...) use. 
	 */
	template <typename T>
	static Eigen::Matrix<T, 4, 4> getLeftQuaternion( const Eigen::Quaternion<T>& q) {
		Eigen::Matrix<T, 4, 4> L = Eigen::Matrix<T, 4, 4>::Zero();
		Eigen::Matrix<T, 3, 1> qvec = q.vec(); 

		L.template block<1, 3>(0, 1) = -qvec.transpose();
		L.template block<3, 1>(1, 0) = qvec;
		L.template block<3, 3>(1, 1) = skewMatrix(qvec);

		L += T( q.w() ) * Eigen::Matrix<T, 4, 4>::Identity();

		return L;
	}

	/**
	 *	@brief	getRightQuaternion
	 *
	 *					returns the matrix for right-multiplying a hamiltonian quaternion.
	 *					We use hamiltonian by default in default templated functions since it is
	 *					what most libraries (Eigen, Ceres, ...) use. 
	 */
	template <typename T>
	static Eigen::Matrix<T, 4, 4> getRightQuaternion( const Eigen::Quaternion<T>& q) {
		Eigen::Matrix<T, 4, 4> R = Eigen::Matrix<T, 4, 4>::Zero();
		Eigen::Matrix<T, 3, 1> qvec = q.vec(); 

		R.template block<1, 3>(0, 1) = -qvec.transpose();
		R.template block<3, 1>(1, 0) = qvec;
		R.template block<3, 3>(1, 1) = -skewMatrix(qvec);

		R += T( q.w() ) * Eigen::Matrix<T, 4, 4>::Identity();

		return R;
	}

	/**
	 *	@brief	getQuatDerivativeMatrix
	 *				
	 *					Returns the 4x3 matrix used to get the quaternion derivative given
	 *					the body rates (from rate gyro)
	 */
	template <typename T>
	static Eigen::Matrix<T, 4, 3> getOmegaMatrix( const Eigen::Quaternion<T>& q ) {

		 Eigen::Matrix<T, 4, 3> Omega = Eigen::Matrix<T, 4, 3>::Zero();

		 Omega << -q.x(), -q.y(), -q.z(), 
		 					 q.w(), -q.z(),  q.y(),
		 					 q.z(),  q.w(), -q.x(), 
		 					-q.y(),  q.x(),  q.w();

		 return Omega;
	}

	/**
	 *	@brief	sign
	 * 
	 * 					returns 1 if positive, -1 if negative, 0 otherwise
	 */
	template<class T>
	constexpr T sign(const T& x) {
	    return ((x) < (0) ? (-1) : ((x) > (0) ? (1) : (0)));
	}

	/**
	 *	@brief	inRange
	 *
	 * 					Returns true if given value x is within the closed interval [x1,x2], 
	 *					otherwise returns false.
	 */
	template<class T>
	constexpr bool inRange(const T &x, const T &x1, const T &x2) {
	    return ((x) <= (x2) && (x) >= (x1) ? true : false);
	}

} // namespace math
} // namespace AutoSM
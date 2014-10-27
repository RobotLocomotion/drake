#ifndef __DRAKE_GEOMETRY_UTIL_H__
#define __DRAKE_GEOMETRY_UTIL_H__

#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <random>

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeGeometryUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
  #include "drakeGradientUtil.h"
#endif

const int TWIST_SIZE = 6;
const int QUAT_SIZE = 4;
const int HOMOGENEOUS_TRANSFORM_SIZE = 16;
const int AXIS_ANGLE_SIZE = 4;
const int SPACE_DIMENSION = 3;
const int RotmatSize = SPACE_DIMENSION * SPACE_DIMENSION;
const int RPY_SIZE = 3;

DLLEXPORT double angleDiff(double phi1, double phi2);

/*
 * quaternion methods
 */
DLLEXPORT Eigen::Vector4d quatConjugate(const Eigen::Vector4d &q);
DLLEXPORT Eigen::Matrix4d dquatConjugate();
DLLEXPORT Eigen::Vector4d quatProduct(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
DLLEXPORT Eigen::Matrix<double, 4, 8> dquatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2);
DLLEXPORT Eigen::Vector3d quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
DLLEXPORT Eigen::Matrix<double, 3, 7> dquatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
DLLEXPORT Eigen::Vector4d quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
DLLEXPORT Eigen::Matrix<double, 4, 8> dquatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
DLLEXPORT double quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);
DLLEXPORT Eigen::Matrix<double, 1, 11> dquatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);
DLLEXPORT double quatNorm(const Eigen::Vector4d& q);
DLLEXPORT Eigen::Vector4d uniformlyRandomAxisAngle(std::default_random_engine& generator);
DLLEXPORT Eigen::Vector4d uniformlyRandomQuat(std::default_random_engine& generator);
DLLEXPORT Eigen::Matrix3d uniformlyRandomRotmat(std::default_random_engine& generator);
DLLEXPORT Eigen::Vector3d uniformlyRandomRPY(std::default_random_engine& generator);

/*
 * quat2x
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> quat2axis(const Eigen::MatrixBase<Derived>& q);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2rotmat(const Eigen::MatrixBase<Derived>& q);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 1> quat2rpy(const Eigen::MatrixBase<Derived>& q);

/*
 * axis2x
 */
template <typename Derived>
DLLEXPORT Eigen::Vector4d axis2quat(const Eigen::MatrixBase<Derived>& a);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> axis2rotmat(const Eigen::MatrixBase<Derived>& a);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 1> axis2rpy(const Eigen::MatrixBase<Derived>& a);

/*
 * rotmat2x
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2axis(const Eigen::MatrixBase<Derived>& R);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2quat(const Eigen::MatrixBase<Derived>& M);

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 1> rotmat2rpy(const Eigen::MatrixBase<Derived>& R);

/*
 * rpy2x
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2axis(const Eigen::MatrixBase<Derived>& rpy);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2quat(const Eigen::MatrixBase<Derived>& rpy);

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy);


template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> vectorToSkewSymmetric(const Eigen::MatrixBase<Derived>& p);

/*
 * rotation conversion gradient functions
 */

#if !defined(WIN32) && !defined(WIN64)
template <typename Derived>
void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 1>::type* dx_norm = nullptr,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 2>::type* ddx_norm = nullptr);


template <typename Derived>
typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Derived>& q);

template <typename DerivedR, typename DerivedDR>
typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, RPY_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2rpy(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR);

template <typename DerivedR, typename DerivedDR>
typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, QUAT_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2quat(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR);

/*
 * angular velocity conversion functions
 */
template <typename DerivedQ, typename DerivedM>
void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr);

template<typename DerivedRPY, typename DerivedPhi>
void angularvel2rpydotMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Gradient<DerivedPhi, RPY_SIZE, 1>::type* dphi = nullptr,
    typename Gradient<DerivedPhi, RPY_SIZE, 2>::type* ddphi = nullptr);

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, SPACE_DIMENSION, RPY_SIZE> rpydot2angularvelMatrix(const Eigen::MatrixBase<Derived>& rpy);

template <typename DerivedQ, typename DerivedM>
void quatdot2angularvelMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr);

/*
 * spatial transform functions
 */
template<typename Scalar, typename DerivedM>
Eigen::Matrix<Scalar, TWIST_SIZE, DerivedM::ColsAtCompileTime> transformSpatialMotion(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M);

template<typename Scalar, typename DerivedF>
Eigen::Matrix<Scalar, TWIST_SIZE, DerivedF::ColsAtCompileTime> transformSpatialForce(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedF>& F);

/*
 * spatial transform gradient methods
 */
template<typename Scalar, typename DerivedS, typename DerivedQdotToV>
Eigen::Matrix<Scalar, HOMOGENEOUS_TRANSFORM_SIZE, DerivedQdotToV::ColsAtCompileTime> dHomogTrans(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedS>& S,
    const Eigen::MatrixBase<DerivedQdotToV>& qdot_to_v);

template<typename Scalar, typename DerivedDT>
Eigen::Matrix<Scalar, HOMOGENEOUS_TRANSFORM_SIZE, DerivedDT::ColsAtCompileTime> dHomogTransInv(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedDT>& dT);

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type dTransformAdjoint(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedX>& X,
    const Eigen::MatrixBase<DerivedDT>& dT,
    const Eigen::MatrixBase<DerivedDX>& dX);

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type dTransformAdjointTranspose(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedX>& X,
    const Eigen::MatrixBase<DerivedDT>& dT,
    const Eigen::MatrixBase<DerivedDX>& dX);
#endif

#endif

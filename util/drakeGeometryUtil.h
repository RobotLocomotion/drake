#ifndef __DRAKE_GEOMETRY_UTIL_H__
#define __DRAKE_GEOMETRY_UTIL_H__

#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <random>
#include "drakeGradientUtil.h"
#include "GradientVar.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeGeometryUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
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

template<typename Scalar>
DLLEXPORT GradientVar<Scalar, Eigen::Dynamic, 1> rotmat2Representation(const GradientVar<Scalar, SPACE_DIMENSION, SPACE_DIMENSION>& R, int rotation_type);

DLLEXPORT int rotationRepresentationSize(int rotation_type);

/*
 * rpy2x
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2axis(const Eigen::MatrixBase<Derived>& rpy);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2quat(const Eigen::MatrixBase<Derived>& rpy);

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy);

/*
 * cross product related
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> vectorToSkewSymmetric(const Eigen::MatrixBase<Derived>& p);

template <typename DerivedA, typename DerivedB>
DLLEXPORT Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b,
    const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename Gradient<DerivedB, Eigen::Dynamic>::type& db);

/*
 * rotation conversion gradient functions
 */

template <typename Derived>
DLLEXPORT void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 1>::type* dx_norm = nullptr,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 2>::type* ddx_norm = nullptr);


template <typename Derived>
DLLEXPORT typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Derived>& q);

template <typename DerivedR, typename DerivedDR>
DLLEXPORT typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, RPY_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2rpy(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR);

template <typename DerivedR, typename DerivedDR>
DLLEXPORT typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, QUAT_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2quat(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR);

/*
 * angular velocity conversion functions
 */
template <typename DerivedQ, typename DerivedM>
DLLEXPORT void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr);

template<typename DerivedRPY, typename DerivedPhi>
DLLEXPORT void angularvel2rpydotMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Gradient<DerivedPhi, RPY_SIZE, 1>::type* dphi = nullptr,
    typename Gradient<DerivedPhi, RPY_SIZE, 2>::type* ddphi = nullptr);

template<typename Scalar>
DLLEXPORT GradientVar<Scalar, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type,
    GradientVar<Scalar, Eigen::Dynamic, 1> qrot);

template<typename DerivedRPY, typename DerivedE>
DLLEXPORT void rpydot2angularvelMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    Eigen::MatrixBase<DerivedE>& E,
    typename Gradient<DerivedE,RPY_SIZE,1>::type* dE=nullptr);

template <typename DerivedQ, typename DerivedM>
DLLEXPORT void quatdot2angularvelMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr);

//#endif

/*
 * spatial transform functions
 */
template<typename Derived>
struct TransformSpatial {
  typedef typename Eigen::Matrix<typename Derived::Scalar, TWIST_SIZE, Derived::ColsAtCompileTime> type;
};

template<typename DerivedM>
DLLEXPORT typename TransformSpatial<DerivedM>::type transformSpatialMotion(
    const Eigen::Transform<typename DerivedM::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M);

template<typename DerivedF>
DLLEXPORT typename TransformSpatial<DerivedF>::type transformSpatialForce(
    const Eigen::Transform<typename DerivedF::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedF>& F);

template<typename DerivedI>
DLLEXPORT GradientVar<typename DerivedI::Scalar, TWIST_SIZE, TWIST_SIZE> transformSpatialInertia(
    const Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>& T_current_to_new,
    const typename Gradient<typename Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>::MatrixType, Eigen::Dynamic>::type* dT_current_to_new,
    const Eigen::MatrixBase<DerivedI>& I);

/*
 * spatial transform gradient methods
 */
template<typename DerivedQdotToV>
struct DHomogTrans {
  typedef typename Eigen::Matrix<typename DerivedQdotToV::Scalar, HOMOGENEOUS_TRANSFORM_SIZE, DerivedQdotToV::ColsAtCompileTime> type;
};
 
template<typename DerivedS, typename DerivedQdotToV>
DLLEXPORT typename DHomogTrans<DerivedQdotToV>::type dHomogTrans(
    const Eigen::Transform<typename DerivedQdotToV::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedS>& S,
    const Eigen::MatrixBase<DerivedQdotToV>& qdot_to_v);

template<typename DerivedDT>
DLLEXPORT typename DHomogTrans<DerivedDT>::type dHomogTransInv(
    const Eigen::Transform<typename DerivedDT::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedDT>& dT);

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
DLLEXPORT typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type dTransformSpatialMotion(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedX>& X,
    const Eigen::MatrixBase<DerivedDT>& dT,
    const Eigen::MatrixBase<DerivedDX>& dX);

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
DLLEXPORT typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type dTransformSpatialForce(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedX>& X,
    const Eigen::MatrixBase<DerivedDT>& dT,
    const Eigen::MatrixBase<DerivedDX>& dX);

#endif

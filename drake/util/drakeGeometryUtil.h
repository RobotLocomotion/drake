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
const int EXPMAP_SIZE = 3;
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
DLLEXPORT Eigen::Vector4d slerp(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, double interpolation_parameter);
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
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2rotmat(const Eigen::MatrixBase<Derived>& q)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto w = q_normalized(0);
  auto x = q_normalized(1);
  auto y = q_normalized(2);
  auto z = q_normalized(3);

  Eigen::Matrix<typename Derived::Scalar, 3, 3> M;
  M.row(0) << w * w + x * x - y * y - z * z, 2.0 * x * y - 2.0 * w * z, 2.0 * x * z + 2.0 * w * y;
  M.row(1) << 2.0 * x * y + 2.0 * w * z, w * w + y * y - x * x - z * z, 2.0 * y * z - 2.0 * w * x;
  M.row(2) << 2.0 * x * z - 2.0 * w * y, 2.0 * y * z + 2.0 * w * x, w * w + z * z - x * x - y * y;

  return M;
}

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 1> quat2rpy(const Eigen::MatrixBase<Derived>& q);

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> quat2eigenQuaternion(const Eigen::MatrixBase<Derived> &q) 
{
  // The Eigen Quaterniond constructor when used with 4 arguments, uses the (w, x, y, z) ordering, just as we do. 
  // HOWEVER: when the constructor is called on a 4-element Vector, the elements must be in (x, y, z, w) order.
  // So, the following two calls will give you the SAME quaternion:
  // Quaternion<double>(q(0), q(1), q(2), q(3));
  // Quaternion<double>(Vector4d(q(3), q(0), q(1), q(2)))
  // which is gross and will cause you much pain.
  // see: http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a91b6ea2cac13ab2d33b6e74818ee1490
  //
  // This method takes a nice, normal (w, x, y, z) order vector and gives you the Quaternion you expect. 
  return Eigen::Quaternion<typename Derived::Scalar>(q(0), q(1), q(2), q(3));
}

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
 * expmap2x
 */
template <typename Derived>
DLLEXPORT GradientVar<typename Derived::Scalar, QUAT_SIZE, 1> expmap2quat(const Eigen::MatrixBase<Derived>& q, const int gradient_ordr);

/*
 * rotmat2x
 */
template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2axis(const Eigen::MatrixBase<Derived>& R);

template <typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2quat(const Eigen::MatrixBase<Derived>& M);

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 1> rotmat2rpy(const Eigen::MatrixBase<Derived>& R);

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> rotmat2Representation(const Eigen::MatrixBase<Derived>& R, int rotation_type);

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
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0), c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0), s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

  return R;
};

template<typename Derived>
DLLEXPORT Eigen::Matrix<typename Derived::Scalar, 9, 3> drpy2rotmat(const Eigen::MatrixBase<Derived>& rpy);

DLLEXPORT Eigen::Matrix3d rotz(double theta);
DLLEXPORT void rotz(double theta, Eigen::Matrix3d &M, Eigen::Matrix3d &dM, Eigen::Matrix3d &ddM);
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


// NOTE: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
DLLEXPORT void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 1>::type* dx_norm = nullptr,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 2>::type* ddx_norm = nullptr)
{
  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(), (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      typename Derived::Index n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) += column_as_matrix.col(row_block);
        }
      }
    }
  }
}


/*
 * rotation conversion gradient functions
 */
template <typename Derived>
DLLEXPORT typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Derived>& q)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, QUAT_SIZE);

  typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type ret;
  typename Eigen::MatrixBase<Derived>::PlainObject qtilde;
  typename Gradient<Derived, QUAT_SIZE>::type dqtilde;
  normalizeVec(q, qtilde, &dqtilde);

  typedef typename Derived::Scalar Scalar;
  Scalar w=qtilde(0);
  Scalar x=qtilde(1);
  Scalar y=qtilde(2);
  Scalar z=qtilde(3);

  ret << w, x, -y, -z, z, y, x, w, -y, z, -w, x, -z, y, x, -w, w, -x, y, -z, x, w, z, y, y, z, w, x, -x, -w, z, y, w, -x, -y, z;
  ret *= 2.0;
  ret *= dqtilde;
  return ret;
}

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
template <typename DerivedQ, typename DerivedM, typename DerivedDM>
void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    Eigen::MatrixBase<DerivedDM>* dM = nullptr) {
  // note: not normalizing to match MATLAB implementation
  M.resize(QUAT_SIZE, SPACE_DIMENSION);
  M.row(0) << -q(1), -q(2), -q(3);
  M.row(1) << q(0), q(3), -q(2);
  M.row(2) << -q(3), q(0), q(1);
  M.row(3) << q(2), -q(1), q(0);
  M *= 0.5;

  if (dM) {
    (*dM) << 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
        -0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0,
        0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;
  }
};

template<typename DerivedRPY, typename DerivedPhi, typename DerivedDPhi, typename DerivedDDPhi>
void angularvel2rpydotMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Eigen::MatrixBase<DerivedDPhi>* dphi = nullptr,
    typename Eigen::MatrixBase<DerivedDDPhi>* ddphi = nullptr);

template<typename Derived>
DLLEXPORT GradientVar<typename Derived::Scalar, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type, const Eigen::MatrixBase<Derived>& qrot, int gradient_order);

template<typename DerivedRPY, typename DerivedE>
DLLEXPORT void rpydot2angularvelMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    Eigen::MatrixBase<DerivedE>& E,
    typename Gradient<DerivedE,RPY_SIZE,1>::type* dE=nullptr) {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>, RPY_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedE>, SPACE_DIMENSION,RPY_SIZE);
  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar sy = sin(y);
  Scalar cy = cos(y);

  E << cp*cy, -sy, 0.0, cp*sy, cy, 0.0, -sp, 0.0, 1.0;
  if(dE)
  {
      (*dE)<< 0.0, -sp*cy, -cp*sy, 0.0, -sp*sy, cp*cy, 0.0, -cp, 0.0, 0.0, 0.0, -cy, 0.0, 0.0, -sy, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
};

template <typename DerivedQ, typename DerivedM>
DLLEXPORT void quatdot2angularvelMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedQ>, QUAT_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedM>, SPACE_DIMENSION, QUAT_SIZE);

  typename DerivedQ::PlainObject qtilde;
  if (dM) {
    typename Gradient<DerivedQ, QUAT_SIZE>::type dqtilde;
    normalizeVec(q, qtilde, &dqtilde);
    (*dM) << 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0;
    (*dM) *= dqtilde;
  } else {
    normalizeVec(q, qtilde);
  }
  M << -qtilde(1), qtilde(0), -qtilde(3), qtilde(2), -qtilde(2), qtilde(3), qtilde(0), -qtilde(1), -qtilde(3), -qtilde(2), qtilde(1), qtilde(0);
  M *= 2.0;
};

template<typename ScalarT>
DLLEXPORT  void cylindrical2cartesian(const Eigen::Matrix<ScalarT,3,1> &cylinder_axis, const Eigen::Matrix<ScalarT,3,1> &cylinder_x_dir, const Eigen::Matrix<ScalarT,3,1> & cylinder_origin, const Eigen::Matrix<ScalarT,6,1> &x_cylinder, const Eigen::Matrix<ScalarT,6,1> &v_cylinder, Eigen::Matrix<ScalarT,6,1> &x_cartesian, Eigen::Matrix<ScalarT,6,1> &v_cartesian, Eigen::Matrix<ScalarT,6,6> &J, Eigen::Matrix<ScalarT,6,1> &Jdotv );

template<typename ScalarT>
DLLEXPORT  void cartesian2cylindrical(const Eigen::Matrix<ScalarT,3,1> &cylinder_axis, const Eigen::Matrix<ScalarT,3,1> &cylinder_x_dir, const Eigen::Matrix<ScalarT,3,1> & cylinder_origin, const Eigen::Matrix<ScalarT,6,1> &x_cartesian, const Eigen::Matrix<ScalarT,6,1> &v_cartesian, Eigen::Matrix<ScalarT,6,1> &x_cylinder, Eigen::Matrix<ScalarT,6,1> &v_cylinder, Eigen::Matrix<ScalarT,6,6> &J, Eigen::Matrix<ScalarT,6,1> &Jdotv );
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

template<typename DerivedA, typename DerivedB>
DLLEXPORT typename TransformSpatial<DerivedB>::type crossSpatialMotion(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b);

template<typename DerivedA, typename DerivedB>
typename TransformSpatial<DerivedB>::type crossSpatialForce(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b);

template<typename DerivedA, typename DerivedB>
DLLEXPORT Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialMotion(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b,
  const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
  const typename Gradient<DerivedB, Eigen::Dynamic>::type& db);

template<typename DerivedA, typename DerivedB>
DLLEXPORT Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialForce(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b,
  const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
  const typename Gradient<DerivedB, Eigen::Dynamic>::type& db);

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

DLLEXPORT GradientVar<double,3,1> quat2expmap(const Eigen::Ref<const Eigen::Vector4d> &q, int gradient_order);
 
DLLEXPORT GradientVar<double,3,1> flipExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap, int gradient_order);

DLLEXPORT GradientVar<double,3,1> unwrapExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap1, const Eigen::Ref<const Eigen::Vector3d> &expmap2, int gradient_order);

DLLEXPORT void quat2expmapSequence(const Eigen::Ref<const Eigen::Matrix<double,4,Eigen::Dynamic>> &quat, const Eigen::Ref<const Eigen::Matrix<double,4,Eigen::Dynamic>> &quat_dot, Eigen::Ref<Eigen::Matrix<double,3,Eigen::Dynamic>> expmap, Eigen::Ref<Eigen::Matrix<double,3,Eigen::Dynamic>> expmap_dot);

DLLEXPORT GradientVar<double,3,1> closestExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap1, const Eigen::Ref<const Eigen::Vector3d> &expmap2, int gradient_order);


#endif

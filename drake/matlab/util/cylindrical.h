#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/common/drake_export.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace util {

namespace internal {
/// Returns a 3D rotation matrix by @p theta about the z axis.
DRAKE_EXPORT Eigen::Matrix3d rotz(double theta);

/// Computes a 3D rotation matrix by @p theta about the z axis.
/// Writes that matrix into @p M, its derivative into @p dM, and its second
/// derivative into @p ddM.
DRAKE_EXPORT void rotz(double theta, Eigen::Matrix3d* M,
                           Eigen::Matrix3d* dM, Eigen::Matrix3d* ddM);
}  // namespace internal

template <typename Scalar>
void cylindrical2cartesian(const Vector3<Scalar>& m_cylinder_axis,
                           const Vector3<Scalar>& m_cylinder_x_dir,
                           const Vector3<Scalar>& cylinder_origin,
                           const Vector6<Scalar>& x_cylinder,
                           const Vector6<Scalar>& v_cylinder,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& x_cartesian,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& v_cartesian, Matrix6<Scalar>& J,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& Jdotv) {
  using std::cos;
  using std::sin;
  Vector3<Scalar> cylinder_axis = m_cylinder_axis / m_cylinder_axis.norm();
  Vector3<Scalar> cylinder_x_dir = m_cylinder_x_dir / m_cylinder_x_dir.norm();
  Matrix3<Scalar> R_cylinder2cartesian;
  R_cylinder2cartesian.col(0) = cylinder_x_dir;
  R_cylinder2cartesian.col(1) = cylinder_axis.cross(cylinder_x_dir);
  R_cylinder2cartesian.col(2) = cylinder_axis;
  double radius = x_cylinder(0);
  double theta = x_cylinder(1);
  double c_theta = cos(theta);
  double s_theta = sin(theta);
  double height = x_cylinder(2);
  double radius_dot = v_cylinder(0);
  double theta_dot = v_cylinder(1);
  double height_dot = v_cylinder(2);
  Vector3<Scalar> x_pos_cartesian;
  x_pos_cartesian << radius * c_theta, radius * s_theta, height;
  x_pos_cartesian = R_cylinder2cartesian * x_pos_cartesian + cylinder_origin;
  Vector3<Scalar> v_pos_cartesian;
  v_pos_cartesian << radius * -s_theta * theta_dot + radius_dot * c_theta,
      radius * c_theta * theta_dot + radius_dot * s_theta, height_dot;
  v_pos_cartesian = R_cylinder2cartesian * v_pos_cartesian;
  Eigen::Vector3d x_rpy_cylinder = x_cylinder.block(3, 0, 3, 1);
  Matrix3<Scalar> R_tangent = math::rpy2rotmat(x_rpy_cylinder);
  Matrix3<Scalar> R_tangent2cylinder;
  Matrix3<Scalar> dR_tangent2cylinder;
  Matrix3<Scalar> ddR_tangent2cylinder;
  internal::rotz(theta - M_PI / 2, &R_tangent2cylinder, &dR_tangent2cylinder,
                 &ddR_tangent2cylinder);
  Matrix3<Scalar> dR_tangent2cylinder_dtheta = dR_tangent2cylinder;
  Matrix3<Scalar> R_cylinder = R_tangent2cylinder * R_tangent;
  Matrix3<Scalar> R_cartesian = R_cylinder2cartesian * R_cylinder;
  Vector3<Scalar> x_rpy_cartesian = math::rotmat2rpy(R_cartesian);
  x_cartesian.block(0, 0, 3, 1) = x_pos_cartesian;
  x_cartesian.block(3, 0, 3, 1) = x_rpy_cartesian;
  v_cartesian.block(0, 0, 3, 1) = v_pos_cartesian;
  v_cartesian.block(3, 0, 3, 1) =
      theta_dot * R_cylinder2cartesian.col(2) +
      R_cylinder2cartesian * R_tangent2cylinder * v_cylinder.block(3, 0, 3, 1);
  J = Matrix6<Scalar>::Zero();
  J.block(0, 0, 3, 1) << c_theta, s_theta, 0;
  J.block(0, 1, 3, 1) << radius * -s_theta, radius * c_theta, 0;
  J.block(0, 2, 3, 1) << 0, 0, 1;
  J.block(0, 0, 3, 3) = R_cylinder2cartesian * J.block(0, 0, 3, 3);
  J.block(3, 1, 3, 1) = R_cylinder2cartesian.col(2);
  J.block(3, 3, 3, 3) = R_cylinder2cartesian * R_tangent2cylinder;
  Matrix3<Scalar> dJ1_dradius = Matrix3<Scalar>::Zero();
  dJ1_dradius(0, 1) = -s_theta;
  dJ1_dradius(1, 1) = c_theta;
  Matrix3<Scalar> dJ1_dtheta = Matrix3<Scalar>::Zero();
  dJ1_dtheta(0, 0) = -s_theta;
  dJ1_dtheta(0, 1) = -radius * c_theta;
  dJ1_dtheta(1, 0) = c_theta;
  dJ1_dtheta(1, 1) = -radius * s_theta;
  Jdotv.block(0, 0, 3, 1) = R_cylinder2cartesian * (dJ1_dradius * radius_dot +
                                                    dJ1_dtheta * theta_dot) *
                            v_cylinder.block(0, 0, 3, 1);
  Jdotv.block(3, 0, 3, 1) = R_cylinder2cartesian * dR_tangent2cylinder_dtheta *
                            theta_dot * v_cylinder.block(3, 0, 3, 1);
}

template <typename Scalar>
void cartesian2cylindrical(const Vector3<Scalar>& m_cylinder_axis,
                           const Vector3<Scalar>& m_cylinder_x_dir,
                           const Vector3<Scalar>& cylinder_origin,
                           const Vector6<Scalar>& x_cartesian,
                           const Vector6<Scalar>& v_cartesian,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& x_cylinder,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& v_cylinder, Matrix6<Scalar>& J,
                           // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references)
                           Vector6<Scalar>& Jdotv) {
  using std::atan2;
  using std::pow;
  using std::sqrt;
  Vector3<Scalar> cylinder_axis = m_cylinder_axis / m_cylinder_axis.norm();
  Vector3<Scalar> cylinder_x_dir = m_cylinder_x_dir / m_cylinder_x_dir.norm();
  Matrix3<Scalar> R_cylinder2cartesian;
  R_cylinder2cartesian.col(0) = cylinder_x_dir;
  R_cylinder2cartesian.col(1) = cylinder_axis.cross(cylinder_x_dir);
  R_cylinder2cartesian.col(2) = cylinder_axis;
  Matrix3<Scalar> R_cartesian2cylinder = R_cylinder2cartesian.transpose();
  Vector3<Scalar> x_pos_cylinder =
      R_cartesian2cylinder * (x_cartesian.block(0, 0, 3, 1) - cylinder_origin);
  Vector3<Scalar> v_pos_cylinder =
      R_cartesian2cylinder * v_cartesian.block(0, 0, 3, 1);
  double radius = sqrt(pow(x_pos_cylinder(0), 2) + pow(x_pos_cylinder(1), 2));
  double radius_dot = (x_pos_cylinder(0) * v_pos_cylinder(0) +
                       x_pos_cylinder(1) * v_pos_cylinder(1)) /
                      radius;
  double theta = atan2(x_pos_cylinder(1), x_pos_cylinder(0));
  double radius_square = pow(radius, 2);
  double radius_cubic = pow(radius, 3);
  double radius_quad = pow(radius, 4);
  double theta_dot = (-x_pos_cylinder(1) * v_pos_cylinder(0) +
                      x_pos_cylinder(0) * v_pos_cylinder(1)) /
                     radius_square;
  double height = x_pos_cylinder(2);
  double height_dot = v_pos_cylinder(2);
  x_cylinder(0) = radius;
  x_cylinder(1) = theta;
  x_cylinder(2) = height;
  v_cylinder(0) = radius_dot;
  v_cylinder(1) = theta_dot;
  v_cylinder(2) = height_dot;
  Matrix3<Scalar> R_tangent2cylinder;
  Matrix3<Scalar> dR_tangent2cylinder;
  Matrix3<Scalar> ddR_tangent2cylinder;
  internal::rotz(theta - M_PI / 2, &R_tangent2cylinder, &dR_tangent2cylinder,
                 &ddR_tangent2cylinder);
  Matrix3<Scalar> R_cylinder2tangent = R_tangent2cylinder.transpose();
  Eigen::Vector3d x_rpy_cartesian = x_cartesian.block(3, 0, 3, 1);
  Matrix3<Scalar> R_cartesian = math::rpy2rotmat(x_rpy_cartesian);
  x_cylinder.block(3, 0, 3, 1) =
      math::rotmat2rpy(R_cylinder2tangent * R_cartesian2cylinder * R_cartesian);
  J = Matrix6<Scalar>::Zero();
  Matrix6<Scalar> Jdot = Matrix6<Scalar>::Zero();
  J(0, 0) = x_pos_cylinder(0) / radius;
  J(0, 1) = x_pos_cylinder(1) / radius;
  J(1, 0) = -x_pos_cylinder(1) / radius_square;
  J(1, 1) = x_pos_cylinder(0) / radius_square;
  J(2, 2) = 1.0;
  J.block(0, 0, 3, 3) = J.block(0, 0, 3, 3) * R_cartesian2cylinder;
  Jdot(0, 0) =
      pow(x_pos_cylinder(1), 2) / radius_cubic * v_pos_cylinder(0) -
      x_pos_cylinder(0) * x_pos_cylinder(1) / radius_cubic * v_pos_cylinder(1);
  Jdot(0, 1) = -x_pos_cylinder(0) * x_pos_cylinder(1) / radius_cubic *
                   v_pos_cylinder(0) +
               pow(x_pos_cylinder(0), 2) / radius_cubic * v_pos_cylinder(1);
  Jdot(1, 0) = 2 * x_pos_cylinder(0) * x_pos_cylinder(1) / radius_quad *
                   v_pos_cylinder(0) +
               (pow(x_pos_cylinder(1), 2) - pow(x_pos_cylinder(0), 2)) /
                   radius_quad * v_pos_cylinder(1);
  Jdot(1, 1) = (pow(x_pos_cylinder(1), 2) - pow(x_pos_cylinder(0), 2)) /
                   radius_quad * v_pos_cylinder(0) -
               2 * x_pos_cylinder(0) * x_pos_cylinder(1) / radius_quad *
                   v_pos_cylinder(1);
  Jdot.block(0, 0, 3, 3) = Jdot.block(0, 0, 3, 3) * R_cartesian2cylinder;
  v_cylinder.block(3, 0, 3, 1) = R_cylinder2tangent * R_cartesian2cylinder *
                                     v_cartesian.block(3, 0, 3, 1) -
                                 theta_dot * R_cylinder2tangent.col(2);
  J.block(3, 0, 3, 3) = R_cylinder2tangent.col(2) * -J.block(1, 0, 1, 3);
  J.block(3, 3, 3, 3) = R_cylinder2tangent * R_cartesian2cylinder;
  Jdot.block(3, 0, 3, 3) = dR_tangent2cylinder.row(2).transpose() *
                               -J.block(1, 0, 1, 3) * theta_dot +
                           R_cylinder2tangent.col(2) * -Jdot.block(1, 0, 1, 3);
  Jdot.block(3, 3, 3, 3) =
      dR_tangent2cylinder.transpose() * theta_dot * R_cartesian2cylinder;
  Jdotv = Jdot * v_cartesian;
}

}  // namespace util
}  // namespace drake

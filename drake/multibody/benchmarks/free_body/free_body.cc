#include "drake/multibody/benchmarks/free_body/free_body.h"

#include <cmath>
#include <tuple>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace benchmarks {
namespace free_body {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaterniond;

std::tuple<Quaterniond, Vector3d, Vector3d>
FreeBody::CalculateExactRotationalSolutionABInitiallyAligned(
    const double t) const {
  // Constant values of moments of inertia.
  const double I = get_I();
  const double J = get_J();

  // Initial values of wx, wy, wz.
  const Vector3d& w_NB_B_initial = get_w_NB_B_initial();
  const double wx0 = w_NB_B_initial[0];
  const double wy0 = w_NB_B_initial[1];
  const double wz0 = w_NB_B_initial[2];

  // Intermediate calculations for quaternion solution.
  const double p =
      std::sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = std::sin(p * t / 2);
  const double cpt2 = std::cos(p * t / 2);
  const double sst2 = std::sin(s * t / 2);
  const double cst2 = std::cos(s * t / 2);

  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is a set Ax, Ay, Az of right-handed orthogonal unit vectors which
  // are fixed in N (Newtonian frame/World) and initially aligned to Bx, By, Bz,
  // where Bz is parallel to B's symmetry axis.
  // Kane produced an analytical solution for quat_AB [eAB0, eAB1, eAB2, eAB3],
  // which allows for direct calculation of the R_AB rotation matrix.
  const double eAB1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double eAB2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double eAB3 = coef * spt2 * cst2 + cpt2 * sst2;
  const double eAB0 = -coef * spt2 * sst2 + cpt2 * cst2;
  const Quaterniond quat_AB(eAB0, eAB1, eAB2, eAB3);

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx = wx0 * std::cos(s * t) + wy0 * std::sin(s * t);
  const double wy = -wx0 * std::sin(s * t) + wy0 * std::cos(s * t);
  const double wz = wz0;

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt = (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  return std::make_tuple(
      quat_AB, Vector3d(wx, wy, wz), Vector3d(wxDt, wyDt, wzDt));
}

std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d>
FreeBody::CalculateExactRotationalSolutionNB(const double t) const {
  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is another set Ax, Ay, Az of right-handed orthogonal unit vectors
  // which are fixed in N (Newtonian frame) but initially aligned to Bx, By, Bz.
  Quaterniond quat_AB;
  Vector3d w_NB_B, wDt_NB_B;
  std::tie(quat_AB, w_NB_B, wDt_NB_B) =
      CalculateExactRotationalSolutionABInitiallyAligned(t);

  // Define A basis as World-fixed basis aligned with B's initial orientation.
  // Multiply (Hamilton product) the quaternions quat_NA * quat_AB to produce
  // quat_NB, which is analogous to multiplying rotation matrices R_NA * R_AB
  // to produce the rotation matrix R_NB.
  // In other words, account for quat_NB_initial (which is quat_NA) to calculate
  // the quaternion quat_NB that is analogous to the R_NB rotation matrix.
  const Quaterniond& quat_NA = get_quat_NB_initial();
  const Quaterniond quat_NB = quat_NA * quat_AB;

  // Analytical solution for time-derivative quaternion in B.
  const Vector4d quatDt =
      math::CalculateQuaternionDtFromAngularVelocityExpressedInB(quat_NB,
                                                                 w_NB_B);

  // Create a tuple to package for returning.
  std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d> returned_tuple;
  std::get<0>(returned_tuple) = quat_NB;
  std::get<1>(returned_tuple) = quatDt;
  std::get<2>(returned_tuple) = w_NB_B;
  std::get<3>(returned_tuple) = wDt_NB_B;

  return returned_tuple;
}

std::tuple<Vector3d, Vector3d, Vector3d>
FreeBody::CalculateExactTranslationalSolution(const double t) const {
  // Initial values of x, y, z and ẋ, ẏ, ż.
  const Vector3d& xyz_initial = get_p_NoBcm_N_initial();
  const Vector3d& xyzDt_initial =
      GetInitialVelocityOfBcmInWorldExpressedInWorld();
  const double x0 = xyz_initial[0];
  const double y0 = xyz_initial[1];
  const double z0 = xyz_initial[2];
  const double xDt0 = xyzDt_initial[0];
  const double yDt0 = xyzDt_initial[1];
  const double zDt0 = xyzDt_initial[2];

  // Analytical solution for ẋ, ẏ, z̈ (only gravitational forces on body).
  const Vector3d& gravity = get_uniform_gravity_expressed_in_world();
  const double x_acceleration = gravity[0];
  const double y_acceleration = gravity[1];
  const double z_acceleration = gravity[2];

  // Analytical solution for ẋ, ẏ, ż (since acceleration is constant).
  const double xDt = xDt0 + x_acceleration * t;
  const double yDt = yDt0 + y_acceleration * t;
  const double zDt = zDt0 + z_acceleration * t;

  // Analytical solution for x, y, z (since acceleration is constant).
  const double x = x0 + xDt0 * t + 0.5 * x_acceleration * t * t;
  const double y = y0 + yDt0 * t + 0.5 * y_acceleration * t * t;
  const double z = z0 + zDt0 * t + 0.5 * z_acceleration * t * t;

  // Create a tuple to package for returning.
  std::tuple<Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector3d& xyz = std::get<0>(returned_tuple);
  Vector3d& xyzDt = std::get<1>(returned_tuple);
  Vector3d& xyzDDt = std::get<2>(returned_tuple);

  // Fill returned_tuple with results.
  xyz << x, y, z;
  xyzDt << xDt, yDt, zDt;
  xyzDDt << x_acceleration, y_acceleration, z_acceleration;

  return returned_tuple;
}

}  // namespace free_body
}  // namespace benchmarks
}  // namespace drake

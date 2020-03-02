#include "drake/multibody/benchmarks/free_body/free_body.h"

#include <cmath>
#include <tuple>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace multibody {
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
  const Vector3d& initial_w_NB_B = get_initial_w_NB_B();
  const double wx0 = initial_w_NB_B[0];
  const double wy0 = initial_w_NB_B[1];
  const double wz0 = initial_w_NB_B[2];

  // Intermediate calculations for quaternion solution.
  double s, p;
  std::tie(s, p) = CalcAngularRates_s_p();
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
  const Vector3d w_NB_B(wx, wy, wz);

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt = (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;
  const Vector3d wDt_NB_B(wxDt, wyDt, wzDt);  // Same as alf_NB_B.

  return std::make_tuple(quat_AB, w_NB_B, wDt_NB_B);
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
  // In other words, account for initial_quat_NB (which is quat_NA) to calculate
  // the quaternion quat_NB that is analogous to the R_NB rotation matrix.
  const Quaterniond& quat_NA = get_initial_quat_NB();
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
  const Vector3d& initial_xyz = get_initial_p_NoBcm_N();
  const Vector3d& initial_xyzDt = CalcInitial_v_NBcm_N();

  // Create a tuple to package for returning the results.
  std::tuple<Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector3d& xyz = std::get<0>(returned_tuple);
  Vector3d& xyzDt = std::get<1>(returned_tuple);
  Vector3d& xyzDDt = std::get<2>(returned_tuple);

  // Analytical solution for ẍ, ÿ, z̈ (only gravitational forces on body).
  const Vector3d& gravity = get_uniform_gravity_expressed_in_world();
  xyzDDt << gravity;

  // Analytical solution for ẋ, ẏ, ż (since acceleration is constant).
  // ẋ = ẋ(0) + ẍ * t,   where ẍ = 0.
  // ẏ = ẏ(0) + ÿ * t,   where ÿ = 0.
  // ż = ż(0) + z̈ * t,   where z̈ = gravity.
  xyzDt << initial_xyzDt + t * gravity;

  // Analytical solution for x, y, z (since acceleration is constant).
  // x = x(0) + ẋ(0) * t + 1/2 * ẍ * t^2,   where ẍ = 0.
  // y = y(0) + ẏ(0) * t + 1/2 * ÿ * t^2,   where ÿ = 0.
  // z = z(0) + ż(0) * t + 1/2 * z̈ * t^2,   where z̈ = gravity.
  xyz << initial_xyz + t * initial_xyzDt + 0.5 * t * t * gravity;

  return returned_tuple;
}

}  // namespace free_body
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

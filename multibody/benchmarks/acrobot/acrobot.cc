#include "drake/multibody/benchmarks/acrobot/acrobot.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace benchmarks {

using Eigen::AutoDiffScalar;
using math::RigidTransform;

template <typename T>
Acrobot<T>::Acrobot(const Vector3<T>& normal, const Vector3<T>& up,
                    double m1, double m2, double l1, double l2,
                    double lc1, double lc2, double Ic1, double Ic2,
                    double b1, double b2, double g)
    : m1_(m1),
      m2_(m2),
      l1_(l1),
      l2_(l2),
      lc1_(lc1),
      lc2_(lc2),
      Ic1_(Ic1),
      Ic2_(Ic2),
      b1_(b1),
      b2_(b2),
      g_(g) {
  // Asserts that neither normal nor up are zero vectors.
  DRAKE_ASSERT(!normal.isZero());
  DRAKE_ASSERT(!up.isZero());
  // Asserts that normal and up are not in the same line.
  DRAKE_ASSERT(
      std::abs(std::abs(ExtractDoubleOrThrow(normal.dot(up))) - 1.0) >
          Eigen::NumTraits<double>::epsilon());

  Vector3<T> z_W = normal.normalized();
  Vector3<T> y_W = (up - up.dot(z_W) * z_W).normalized();
  Vector3<T> x_W = y_W.cross(z_W);
  // Rotation transformation from the model frame D to the world frame W.
  const math::RotationMatrix<T> R_WD =
      math::RotationMatrix<T>::MakeFromOrthonormalColumns(x_W, y_W, z_W);
  X_WD_.set_rotation(R_WD);
}

template <typename T>
Matrix2<T> Acrobot<T>::CalcMassMatrix(const T& theta2) const {
  const T I1 = Ic1_ + m1_ * lc1_ * lc1_;
  const T I2 = Ic2_ + m2_ * lc2_ * lc2_;
  const T m2l1lc2 = m2_ * l1_ * lc2_;  // occurs often!

  using std::sin;
  using std::cos;
  const T c2 = cos(theta2);

  const T h12 = I2 + m2l1lc2 * c2;
  Matrix2<T> H;
  H << I1 + I2 + m2_ * l1_ * l1_ + 2 * m2l1lc2 * c2, h12, h12, I2;
  return H;
}

template <typename T>
Vector2<T> Acrobot<T>::CalcCoriolisVector(
    const T&, const T& theta2, const T& theta1dot, const T& theta2dot) const {
  using std::sin;
  using std::cos;

  const T s2 = sin(theta2);
  const T m2l1lc2 = m2_ * l1_ * lc2_;  // occurs often!

  Vector2<T> C;
  C << -2 * m2l1lc2 * s2 * theta2dot * theta1dot +
      -m2l1lc2 * s2 * theta2dot * theta2dot,
      m2l1lc2 * s2 * theta1dot * theta1dot;

  return C;
}

template <typename T>
Vector2<T> Acrobot<T>::CalcGravityVector(
    const T& theta1, const T& theta2) const {
  using std::sin;
  using std::cos;

  const T s1 = sin(theta1);
  const T s12 = sin(theta1 + theta2);
  Vector2<T> tau_g;
  tau_g(0) = g_ * m1_ * lc1_ * s1 + g_ * m2_ * (l1_ * s1 + lc2_ * s12);
  tau_g(1) = g_ * m2_ * lc2_ * s12;

  // Unlike http://underactuated.mit.edu/underactuated.html?chapter=3, we define
  // tau_g(q) to be on the right hand side of the equations of motion, that is,
  // Mv̇ + C(q, v)v = tau_g(q).
  // Therefore we invert the sign before returning.
  return -tau_g;
}

template <typename T>
math::RigidTransform<T> Acrobot<T>::CalcLink1PoseInWorldFrame(
    const T& theta1) const {

  using std::sin;
  using std::cos;

  // Center of mass position of link 1 in the model frame D.
  const Vector3<T> xcm1_D = lc1_ * Vector3<T>(sin(theta1), -cos(theta1), 0.0);

  // Pose of link 1 frame measured and expressed in D.
  const math::RigidTransform<T> X_DL1(
      math::RotationMatrix<T>::MakeZRotation(theta1), xcm1_D);

  // Transformation to world frame W.
  return X_WD_ * X_DL1;
}

template <typename T>
RigidTransform<T> Acrobot<T>::CalcLink2PoseInWorldFrame(
    const T& theta1, const T& theta2) const {
  using std::sin;
  using std::cos;

  // Center of mass position of link 2 in the model frame D.
  const Vector3<T> xcm2_D =
      lc2_ * Vector3<T>(sin(theta1 + theta2), -cos(theta1 + theta2), 0.0) +
          l1_ * Vector3<T>(sin(theta1), -cos(theta1), 0.0);

  // Pose of link 2 frame measured and expressed in D.
  const RigidTransform<T> X_DL2(
      math::RotationMatrix<T>::MakeZRotation(theta1 + theta2), xcm2_D);

  // Transformation to world frame W.
  return X_WD_ * X_DL2;
}

template <typename T>
RigidTransform<T> Acrobot<T>::CalcElbowOutboardFramePoseInWorldFrame(
    const T& theta1, const T& theta2) const {
  // Pose of link2's frame L2cm, at the com, in the world frame.
  const RigidTransform<T> X_WL2cm = CalcLink2PoseInWorldFrame(theta1, theta2);
  // Pose of the elbow outboard frame Eo in Lcm's frame.
  // Link 2 is a bar with its axial axis aligned with its frame y-axis.
  // Therefore the elbow outboard frame is at y = +lc2 from link 2's center of
  // mass. See this class's documentation for details.
  const RigidTransform<T> X_L2cmEo(Vector3<T>(0.0, lc2_, 0.0));
  return X_WL2cm * X_L2cmEo;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink1SpatialVelocityInWorldFrame(
    const T& theta1, const T& theta1dot) const {
  using std::sin;
  using std::cos;

  // Linear velocity of link1's center of mass expressed in model frame D.
  const Vector3<T> vcm1_D =
      lc1_ * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dot;

  // Angular velocity of link1 expressed in model frame D.
  const Vector3<T> wcm1_D = Vector3<T>::UnitZ() * theta1dot;

  // Spatial velocity expressed in the world frame.
  Vector6<T> V_WL1;
  V_WL1.template topRows<3>() = X_WD_.rotation() * wcm1_D;
  V_WL1.template bottomRows<3>() = X_WD_.rotation() * vcm1_D;

  return V_WL1;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink2SpatialVelocityInWorldFrame(
    const T& theta1, const T& theta2,
    const T& theta1dot, const T& theta2dot) const {
  using std::sin;
  using std::cos;

  // Linear velocity of link2's center of mass expressed in model frame D.
  const Vector3<T> vcm2_D =
      lc2_ * Vector3<T>(cos(theta1 + theta2), sin(theta1 + theta2), 0.0) *
          (theta1dot + theta2dot) +
      l1_ * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dot;

  // Angular velocity of link2 expressed in model frame D.
  const Vector3<T> wcm2_D = Vector3<T>::UnitZ() * (theta1dot + theta2dot);

  // Spatial velocity expressed in the world frame.
  Vector6<T> V_WL2;
  V_WL2.template topRows<3>() = X_WD_.rotation() * wcm2_D;
  V_WL2.template bottomRows<3>() = X_WD_.rotation() * vcm2_D;

  return V_WL2;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink1SpatialAccelerationInWorldFrame(
    const T& theta1, const T& theta1dot, const T& theta1dotdot) const {
  using std::sin;
  using std::cos;

  // Linear acceleration of link1's center of mass expressed in model frame D.
  const Vector3<T> acm1_D =
      lc1_ * Vector3<T>(-sin(theta1), cos(theta1), 0.0) * theta1dot * theta1dot
          + lc1_ * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dotdot;

  // Angular acceleration of link1 expressed in model frame D.
  const Vector3<T> alphacm1_D = Vector3<T>::UnitZ() * theta1dotdot;

  // Spatial acceleration expressed in the world frame.
  Vector6<T> A_WL1;
  A_WL1.template topRows<3>() = X_WD_.rotation() * alphacm1_D;
  A_WL1.template bottomRows<3>() = X_WD_.rotation() * acm1_D;

  return A_WL1;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink2SpatialAccelerationInWorldFrame(
    const T& theta1, const T& theta2,
    const T& theta1dot, const T& theta2dot,
    const T& theta1dotdot, const T& theta2dotdot) const {
  using std::sin;
  using std::cos;

  // Linear acceleration of link2's center of mass expressed in model frame D.
  const Vector3<T> acm2_D =
      lc2_ * Vector3<T>(-sin(theta1 + theta2), cos(theta1 + theta2), 0.0) *
          (theta1dot + theta2dot) * (theta1dot + theta2dot) +
      lc2_ * Vector3<T>(cos(theta1 + theta2), sin(theta1 + theta2), 0.0) *
          (theta1dotdot + theta2dotdot) +
      l1_ * Vector3<T>(-sin(theta1), cos(theta1), 0.0) * theta1dot * theta1dot +
      l1_ * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dotdot;

  // Angular acceleration of link2 expressed in model frame D.
  Vector3<T> alphacm2_D = Vector3<T>::UnitZ() * (theta1dotdot + theta2dotdot);

  // Spatial acceleration expressed in the world frame.
  Vector6<T> A_WL2;
  A_WL2.template topRows<3>() = X_WD_.rotation() * alphacm2_D;
  A_WL2.template bottomRows<3>() = X_WD_.rotation() * acm2_D;
  return A_WL2;
}

template <typename T>
T Acrobot<T>::CalcPotentialEnergy(const T& theta1, const T& theta2) const {
  const RigidTransform<T> X_WL1cm = CalcLink1PoseInWorldFrame(theta1);
  const RigidTransform<T> X_WL2cm = CalcLink2PoseInWorldFrame(theta1, theta2);
  const Vector3<T>& p_WL1cm = X_WL1cm.translation();
  const Vector3<T>& p_WL2cm = X_WL2cm.translation();
  return (m1_ * p_WL1cm.y() + m2_ * p_WL2cm.y()) * g_;
}

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::benchmarks::Acrobot)

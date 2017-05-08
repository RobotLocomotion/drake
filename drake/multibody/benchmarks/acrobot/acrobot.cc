#include "drake/multibody/benchmarks/acrobot/acrobot.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace benchmarks {

using Eigen::AutoDiffScalar;

template <typename T>
Acrobot<T>::Acrobot(const Vector3<T>& normal, const Vector3<T>& up) {
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
  Matrix3<T> R_WD;
  R_WD.col(0) = x_W;
  R_WD.col(1) = y_W;
  R_WD.col(2) = z_W;
  X_WD_.linear() = R_WD;
}

template <typename T>
Matrix2<T> Acrobot<T>::CalcMassMatrix(const T& theta2) const {
  const T I1 = Ic1 + m1 * lc1 * lc1;
  const T I2 = Ic2 + m2 * lc2 * lc2;
  const T m2l1lc2 = m2 * l1 * lc2;  // occurs often!

  using std::sin;
  using std::cos;
  const T c2 = cos(theta2);

  const T h12 = I2 + m2l1lc2 * c2;
  Matrix2<T> H;
  H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;
  return H;
}

template <typename T>
Isometry3<T> Acrobot<T>::CalcLink1PoseInWorldFrame(
    const T& theta1) const {

  using std::sin;
  using std::cos;

  // Center of mass position of link 1 in the model frame D.
  Vector3<T> xcm1_D = lc1 * Vector3<T>(sin(theta1), -cos(theta1), 0.0);

  // Pose of link 1 frame measured and expressed in D.
  Isometry3<T> X_DL1;
  X_DL1.linear() = Matrix3<T>(AngleAxis<T>(theta1, Vector3<T>::UnitZ()));
  X_DL1.translation() = xcm1_D;

  // Transformation to world frame W.
  return X_WD_ * X_DL1;
}

template <typename T>
Isometry3<T> Acrobot<T>::CalcLink2PoseInWorldFrame(
    const T& theta1, const T& theta2) const {
  using std::sin;
  using std::cos;

  // Center of mass position of link 2 in the model frame D.
  Vector3<T> xcm2_D =
      lc2 * Vector3<T>(sin(theta1 + theta2), -cos(theta1 + theta2), 0.0) +
          l1 * Vector3<T>(sin(theta1), -cos(theta1), 0.0);

  // Pose of link 2 frame measured and expressed in D.
  Isometry3<T> X_DL2;
  X_DL2.linear() =
      Matrix3<T>(AngleAxis<T>(theta1 + theta2, Vector3<T>::UnitZ()));
  X_DL2.translation() = xcm2_D;

  // Transformation to world frame W.
  return X_WD_ * X_DL2;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink1SpatialVelocityInWorldFrame(
    const T& theta1, const T& theta1dot) const {
  using std::sin;
  using std::cos;

  // Linear velocity of link1's center of mass expressed in model frame D.
  Vector3<T> vcm1_D =
      lc1 * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dot;

  // Angular velocity of link1 expressed in model frame D.
  Vector3<T> wcm1_D = Vector3<T>::UnitZ() * theta1dot;

  // Spatial velocity expressed in the world frame.
  Vector6<T> V_WL1;
  V_WL1.template topRows<3>() = X_WD_.linear() * wcm1_D;
  V_WL1.template bottomRows<3>() = X_WD_.linear() * vcm1_D;

  return V_WL1;
}

template <typename T>
Vector6<T> Acrobot<T>::CalcLink2SpatialVelocityInWorldFrame(
    const T& theta1, const T& theta2,
    const T& theta1dot, const T& theta2dot) const {
  using std::sin;
  using std::cos;

  // Linear velocity of link2's center of mass expressed in model frame D.
  Vector3<T> vcm2_D =
      // dx/dtheta1 * theta1dot.
      lc2 * Vector3<T>(
          cos(theta1 + theta2), sin(theta1 + theta2), 0.0) * theta1dot +
      l1 * Vector3<T>(cos(theta1), sin(theta1), 0.0) * theta1dot +
      // dx/dtheta2 * theta2dot.
      lc2 * Vector3<T>(
          cos(theta1 + theta2), sin(theta1 + theta2), 0.0) * theta2dot;

  // Angular velocity of link2 expressed in model frame D.
  Vector3<T> wcm2_D = Vector3<T>::UnitZ() * (theta1dot + theta2dot);

  // Spatial velocity expressed in the world frame.
  Vector6<T> V_WL2;
  V_WL2.template topRows<3>() = X_WD_.linear() * wcm2_D;
  V_WL2.template bottomRows<3>() = X_WD_.linear() * vcm2_D;

  return V_WL2;
}

template class Acrobot<double>;
template class Acrobot<AutoDiffXd>;

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

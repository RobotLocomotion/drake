#pragma once

#include <Eigen/Dense>

#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace rendering {

/// A 6-vector representing the derivatives of the position transform of frame A
/// in the world frame, Xdot_WA, in the form `{R_WA, p_WA}`, where `p` is
/// the derivatives of x-y-z translation, and `R` is the derivatives of x-y-z
/// rotation.
///
/// The exact order of elements is `{ωx, ωy, ωz, vx, vy, vz}`.
///
/// @tparam T The Eigen scalar type. Supported scalar types are double,
///         AutoDiffXd, and symbolic::Expression.
template <typename T>
class FrameVelocity final : public BasicVector<T> {
 public:
  FrameVelocity();
  ~FrameVelocity() override;

  // FrameVelocity is final, so we can implement copy and assignment without
  // fear of object slicing. This is useful for including FrameVelocity in an
  // AbstractValue, such as PoseBundle.
  FrameVelocity(const FrameVelocity<T>& other);
  FrameVelocity<T>& operator=(const FrameVelocity<T>& other);

  /// Returns the entire spatial velocity Xdot_WA.
  multibody::SpatialVelocity<T> get_velocity() const;
  /// Assigns the entire spatial velocity Xdot_WA.
  void set_velocity(const multibody::SpatialVelocity<T>& velocity);

  static constexpr int kSize = 6;

 protected:
  FrameVelocity<T>* DoClone() const override;

 private:
  // Assigns the translational velocity p_WA.
  void set_translational(const Vector3<T>& translational);
  // Assigns the rotational velocity R_WA.
  void set_rotational(const Vector3<T>& rotational);
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake

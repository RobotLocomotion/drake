#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace rendering {

/// A 7-vector representing the transform of frame A in the world frame, X_WA,
/// in the form `{p_WA, R_WA}`, where `p` is represented as an x-y-z
/// translation, and `R` is represented as a quaternion. The exact order of
/// elements is `{x, y, z, qw, qx, qy, qz}`.
///
/// @tparam T The Eigen scalar type. Supported scalar types are double,
///         AutoDiffXd, and symbolic::Expression.
template <typename T>
class PoseVector : public BasicVector<T> {
 public:
  PoseVector();
  ~PoseVector() override;

  /// Returns the transform X_WA.
  Isometry3<T> get_isometry() const;
  // TODO(david-german-tri): Provide set_isometry, once #5274 is resolved.

  /// Returns the translation p_WA.
  Eigen::Translation<T, 3> get_translation() const;
  /// Assigns the translation p_WA.
  void set_translation(const Eigen::Translation<T, 3>& q);

  /// Returns the rotation R_WA.
  Eigen::Quaternion<T> get_rotation() const;
  /// Assigns the rotation R_WA.
  void set_rotation(const Eigen::Quaternion<T>& q);

  static constexpr int kSize = 7;

 protected:
  PoseVector<T>* DoClone() const override;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake

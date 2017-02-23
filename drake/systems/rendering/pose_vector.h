#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace rendering {

/// A 7-vector representing the transform of frame A in the world frame, X_WA,
/// in the form {R_WA, p_WA}, where R is represented as a quaternion, and p as
/// an x-y-z translation.
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

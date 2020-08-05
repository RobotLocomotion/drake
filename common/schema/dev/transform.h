#pragma once

#include <optional>
#include <string>

#include "drake/common/name_value.h"
#include "drake/common/schema/dev/rotation.h"
#include "drake/common/schema/dev/stochastic.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace schema {

/// A specification for a 3d rotation and translation, optionally with respect
/// to a base frame.
struct Transform {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Transform)

  /// Constructs the Identity transform.
  Transform() = default;

  /// Constructs the given transform.
  explicit Transform(const math::RigidTransformd&);

  /// Sets the rotation field to the given deterministic RPY, in degrees.
  void set_rotation_rpy_deg(const Eigen::Vector3d& rpy_deg) {
    rotation.set_rpy_deg(rpy_deg);
  }

  /// Returns true iff this is fully deterministic.
  bool IsDeterministic() const;

  /// If this is deterministic, retrieves its value.
  /// @throws exception if this is not fully deterministic.
  math::RigidTransformd GetDeterministicValue() const;

  /// Returns the symbolic form of this rotation.  If this is deterministic,
  /// the result will contain no variables.  If this is random, the result will
  /// contain one or more random variables, based on the distributions in use.
  math::RigidTransform<symbolic::Expression> ToSymbolic() const;

  /// Returns the mean of this rotation.  If this is deterministic, the result
  /// is the same as GetDeterministicValue.  If this is random, note that the
  /// mean here is simply defined as setting all of the random variables
  /// individually to their mean.  Various other measures of the resulting
  /// RigidTransform (e.g., the distribution of one of the Euler angles) may
  /// not necessarily match that measure on the returned value.
  math::RigidTransformd Mean() const;

  /// Samples this Transform.  If this is deterministic, the result is the same
  /// as GetDeterministicValue.
  math::RigidTransformd Sample(RandomGenerator* random) const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(base_frame));
    a->Visit(DRAKE_NVP(translation));

    // Visit the `rotation` field as if it were optional, to allow it to be
    // missing during a schema transition window.
    // TODO(anzu#4772) Once rotation_rpy_deg is gone, this should be
    // `Visit(MakeNameValue("rotation", &rotation.value))` instead.
    std::optional<Rotation::Variant> maybe_rotation = rotation.value;
    a->Visit(MakeNameValue("rotation", &maybe_rotation));
    if (maybe_rotation) {
      rotation.value = *maybe_rotation;
    }

    // Visit the legacy `rotation_rpy_deg` field as an empty optional, to allow
    // it to be read and obeyed (but not written) during a transition window.
    // TODO(anzu#4772) Remove this compatibility shim once nothing is using it.
    std::optional<Eigen::Vector3d> maybe_rpy;
    a->Visit(MakeNameValue("rotation_rpy_deg", &maybe_rpy));
    if (maybe_rpy) {
      this->set_rotation_rpy_deg(*maybe_rpy);
    }
  }

  /// An optional base frame name for this transform.  When left unspecified,
  /// the default depends on the semantics of the enclosing struct.
  std::optional<std::string> base_frame;

  /// A translation vector, in meters.
  DistributionVectorVariant<3> translation{Eigen::Vector3d::Zero()};

  /// A variant that allows for several ways to specify a rotation.
  Rotation rotation;
};

}  // namespace schema
}  // namespace drake

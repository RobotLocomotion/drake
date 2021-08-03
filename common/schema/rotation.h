#pragma once

#include <variant>

#include "drake/common/name_value.h"
#include "drake/common/schema/stochastic.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace schema {

/// A specification for an SO(3) rotation, to be used for serialization
/// purposes, e.g., to define stochastic scenarios.  This structure specifies
/// either one specific rotation or else a distribution of possible rotations.
/// It does not provide mathematical operators to compose or mutate rotations.
/// Instead, users should call either GetDeterministicValue() or ToSymbolic()
/// to obtain a RotationMatrix value that can be operated on.
///
/// For an overview of configuring stochastic transforms, see
/// @ref schema_transform and @ref schema_stochastic.
///
/// See @ref serialize_tips for implementation details, especially the
/// unusually public member fields.
class Rotation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rotation)

  /// Constructs the Identity rotation.
  Rotation() = default;

  /// Constructs an Rpy rotation with the given value.
  explicit Rotation(const math::RotationMatrix<double>&);

  /// Constructs an Rpy rotation with the given value.
  explicit Rotation(const math::RollPitchYaw<double>&);

  /// No-op rotation.
  struct Identity {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Identity)
    Identity() = default;

    template <typename Archive>
    void Serialize(Archive*) {}
  };

  /// A roll-pitch-yaw rotation, using the angle conventions of Drake's
  /// RollPitchYaw.
  struct Rpy {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rpy)
    Rpy() = default;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(deg));
    }

    DistributionVectorVariant<3> deg{Eigen::Vector3d::Zero()};
  };

  /// Rotation constructed from a fixed axis and an angle.
  struct AngleAxis {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AngleAxis)
    AngleAxis() = default;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(angle_deg));
      a->Visit(DRAKE_NVP(axis));
    }

    DistributionVariant angle_deg;
    DistributionVectorVariant<3> axis{Eigen::Vector3d::UnitZ()};
  };

  /// Rotation sampled from a uniform distribution over SO(3).
  struct Uniform {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Uniform)
    Uniform() = default;

    template <typename Archive>
    void Serialize(Archive*) {}
  };

  /// Returns true iff this is fully deterministic.
  bool IsDeterministic() const;

  /// If this is deterministic, retrieves its value.
  /// @throws std::exception if this is not fully deterministic.
  math::RotationMatrixd GetDeterministicValue() const;

  /// Returns the symbolic form of this rotation.  If this is deterministic,
  /// the result will contain no variables.  If this is random, the result will
  /// contain one or more random variables, based on the distributions in use.
  math::RotationMatrix<symbolic::Expression> ToSymbolic() const;

  /// Sets this value to the given deterministic RPY, in degrees.
  void set_rpy_deg(const Eigen::Vector3d& rpy_deg) {
    value.emplace<Rotation::Rpy>().deg = rpy_deg;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  using Variant = std::variant<Identity, Rpy, AngleAxis, Uniform>;
  Variant value;
};

}  // namespace schema
}  // namespace drake

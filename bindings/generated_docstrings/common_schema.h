#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/common/schema/rotation.h"
// #include "drake/common/schema/stochastic.h"
// #include "drake/common/schema/transform.h"

// Symbol: pydrake_doc_common_schema
constexpr struct /* pydrake_doc_common_schema */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::schema
    struct /* schema */ {
      // Symbol: drake::schema::Deterministic
      struct /* Deterministic */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc = R"""(A single deterministic ``value``.)""";
        // Symbol: drake::schema::Deterministic::Deterministic
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::Deterministic::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::Deterministic::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::Deterministic::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::Deterministic::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::Deterministic::value
        struct /* value */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } value;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("value", value.doc),
          };
        }
      } Deterministic;
      // Symbol: drake::schema::DeterministicVector
      struct /* DeterministicVector */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(A single deterministic vector ``value``.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
        // Symbol: drake::schema::DeterministicVector::DeterministicVector<Size>
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::DeterministicVector::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::DeterministicVector::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::DeterministicVector::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::DeterministicVector::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::DeterministicVector::value
        struct /* value */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } value;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("value", value.doc),
          };
        }
      } DeterministicVector;
      // Symbol: drake::schema::Distribution
      struct /* Distribution */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Base class for a single distribution, to be used with YAML archives.
(See class DistributionVector for vector-valued distributions.)

See implementing_serialize "Implementing Serialize" for implementation
details, especially the unusually public member fields of our
subclasses.)""";
        // Symbol: drake::schema::Distribution::Distribution
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::Distribution::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::Distribution::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::Distribution::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
      } Distribution;
      // Symbol: drake::schema::DistributionVariant
      struct /* DistributionVariant */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Variant over all kinds of distributions.)""";
      } DistributionVariant;
      // Symbol: drake::schema::DistributionVector
      struct /* DistributionVector */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Base class for a vector of distributions, to be used with YAML
archives. (See class Distribution for scalar-valued distributions.)

See implementing_serialize for implementation details, especially the
unusually public member fields in our subclasses.)""";
        // Symbol: drake::schema::DistributionVector::DistributionVector
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::DistributionVector::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::DistributionVector::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::DistributionVector::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
      } DistributionVector;
      // Symbol: drake::schema::DistributionVectorVariantX
      struct /* DistributionVectorVariantX */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(DistributionVectorVariant that permits any vector size dynamically.)""";
      } DistributionVectorVariantX;
      // Symbol: drake::schema::Gaussian
      struct /* Gaussian */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(A gaussian distribution with ``mean`` and ``stddev``.)""";
        // Symbol: drake::schema::Gaussian::Gaussian
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::Gaussian::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::Gaussian::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::Gaussian::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::Gaussian::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::Gaussian::mean
        struct /* mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } mean;
        // Symbol: drake::schema::Gaussian::stddev
        struct /* stddev */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } stddev;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("mean", mean.doc),
            std::make_pair("stddev", stddev.doc),
          };
        }
      } Gaussian;
      // Symbol: drake::schema::GaussianVector
      struct /* GaussianVector */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(A gaussian distribution with vector ``mean`` and vector or scalar
``stddev``.

When ``mean`` and ``stddev`` both have the same number of elements,
that denotes an elementwise pairing of the 0th mean with 0th stddev,
1st mean with 1st stddev, etc.

Alternatively, ``stddev`` can be a vector with a single element, no
matter the size of ``mean``; that denotes the same ``stddev`` value
applied to every element of ``mean``.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
        // Symbol: drake::schema::GaussianVector::GaussianVector<Size>
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::GaussianVector::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::GaussianVector::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::GaussianVector::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::GaussianVector::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::GaussianVector::mean
        struct /* mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } mean;
        // Symbol: drake::schema::GaussianVector::stddev
        struct /* stddev */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } stddev;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("mean", mean.doc),
            std::make_pair("stddev", stddev.doc),
          };
        }
      } GaussianVector;
      // Symbol: drake::schema::GetDeterministicValue
      struct /* GetDeterministicValue */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_var =
R"""(If ``var`` is deterministic, retrieves its value.

Raises:
    RuntimeError if ``var`` is not deterministic.)""";
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_constDistributionVectorVariant =
R"""(If ``vec`` is deterministic, retrieves its value.

Raises:
    RuntimeError if ``vec`` is not deterministic.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
      } GetDeterministicValue;
      // Symbol: drake::schema::IsDeterministic
      struct /* IsDeterministic */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_var =
R"""(Returns true iff ``var`` is set to a deterministic value.)""";
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_constDistributionVectorVariant =
R"""(Returns true iff all of `vec`'s elements are set to a deterministic
value.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
      } IsDeterministic;
      // Symbol: drake::schema::Mean
      struct /* Mean */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_var =
R"""(Like Distribution∷Mean, but on a DistributionVariant instead.)""";
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_vec =
R"""(Like Distribution∷Mean, but elementwise over a collection of
possibly-heterogenous DistributionVariant instead.)""";
      } Mean;
      // Symbol: drake::schema::Rotation
      struct /* Rotation */ {
        // Source: drake/common/schema/rotation.h
        const char* doc =
R"""(A specification for an SO(3) rotation, to be used for serialization
purposes, e.g., to define stochastic scenarios. This structure
specifies either one specific rotation or else a distribution of
possible rotations. It does not provide mathematical operators to
compose or mutate rotations. Instead, users should call either
GetDeterministicValue() or ToSymbolic() to obtain a RotationMatrix
value that can be operated on.

For an overview of configuring stochastic transforms, see
schema_transform and schema_stochastic.

See implementing_serialize "Implementing Serialize" for implementation
details, especially the unusually public member fields.)""";
        // Symbol: drake::schema::Rotation::AngleAxis
        struct /* AngleAxis */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Rotation constructed from a fixed axis and an angle.)""";
          // Symbol: drake::schema::Rotation::AngleAxis::AngleAxis
          struct /* ctor */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::schema::Rotation::AngleAxis::Serialize
          struct /* Serialize */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::schema::Rotation::AngleAxis::angle_deg
          struct /* angle_deg */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } angle_deg;
          // Symbol: drake::schema::Rotation::AngleAxis::axis
          struct /* axis */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } axis;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("angle_deg", angle_deg.doc),
              std::make_pair("axis", axis.doc),
            };
          }
        } AngleAxis;
        // Symbol: drake::schema::Rotation::GetDeterministicValue
        struct /* GetDeterministicValue */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(If this is deterministic, retrieves its value.

Raises:
    RuntimeError if this is not fully deterministic.)""";
        } GetDeterministicValue;
        // Symbol: drake::schema::Rotation::Identity
        struct /* Identity */ {
          // Source: drake/common/schema/rotation.h
          const char* doc = R"""(No-op rotation.)""";
          // Symbol: drake::schema::Rotation::Identity::Identity
          struct /* ctor */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::schema::Rotation::Identity::Serialize
          struct /* Serialize */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } Serialize;
        } Identity;
        // Symbol: drake::schema::Rotation::IsDeterministic
        struct /* IsDeterministic */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Returns true iff this is fully deterministic.)""";
        } IsDeterministic;
        // Symbol: drake::schema::Rotation::Rotation
        struct /* ctor */ {
          // Source: drake/common/schema/rotation.h
          const char* doc_0args = R"""(Constructs the Identity rotation.)""";
          // Source: drake/common/schema/rotation.h
          const char* doc_1args =
R"""(Constructs an Rpy rotation with the given value.)""";
        } ctor;
        // Symbol: drake::schema::Rotation::Rpy
        struct /* Rpy */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(A roll-pitch-yaw rotation, using the angle conventions of Drake's
RollPitchYaw.)""";
          // Symbol: drake::schema::Rotation::Rpy::Rpy
          struct /* ctor */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::schema::Rotation::Rpy::Serialize
          struct /* Serialize */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::schema::Rotation::Rpy::deg
          struct /* deg */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } deg;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("deg", deg.doc),
            };
          }
        } Rpy;
        // Symbol: drake::schema::Rotation::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Samples this Rotation. If this is deterministic, the result is the
same as GetDeterministicValue.)""";
        } Sample;
        // Symbol: drake::schema::Rotation::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/rotation.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::Rotation::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Returns the symbolic form of this rotation. If this is deterministic,
the result will contain no variables. If this is random, the result
will contain one or more random variables, based on the distributions
in use.)""";
        } ToSymbolic;
        // Symbol: drake::schema::Rotation::Uniform
        struct /* Uniform */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Rotation sampled from a uniform distribution over SO(3).)""";
          // Symbol: drake::schema::Rotation::Uniform::Serialize
          struct /* Serialize */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::schema::Rotation::Uniform::Uniform
          struct /* ctor */ {
            // Source: drake/common/schema/rotation.h
            const char* doc = R"""()""";
          } ctor;
        } Uniform;
        // Symbol: drake::schema::Rotation::Variant
        struct /* Variant */ {
          // Source: drake/common/schema/rotation.h
          const char* doc = R"""()""";
        } Variant;
        // Symbol: drake::schema::Rotation::set_rpy_deg
        struct /* set_rpy_deg */ {
          // Source: drake/common/schema/rotation.h
          const char* doc =
R"""(Sets this value to the given deterministic RPY, in degrees.)""";
        } set_rpy_deg;
        // Symbol: drake::schema::Rotation::value
        struct /* value */ {
          // Source: drake/common/schema/rotation.h
          const char* doc = R"""()""";
        } value;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("value", value.doc),
          };
        }
      } Rotation;
      // Symbol: drake::schema::Sample
      struct /* Sample */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc_2args_var_generator =
R"""(Like Distribution∷Sample, but on a DistributionVariant instead.)""";
        // Source: drake/common/schema/stochastic.h
        const char* doc_2args_vec_generator =
R"""(Like Distribution∷Sample, but elementwise over a collection of
possibly-heterogenous DistributionVariant instead.)""";
      } Sample;
      // Symbol: drake::schema::ToDistribution
      struct /* ToDistribution */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Copies the given variant into a Distribution base class.)""";
      } ToDistribution;
      // Symbol: drake::schema::ToDistributionVector
      struct /* ToDistributionVector */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Copies the given variant into a DistributionVector base class.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
      } ToDistributionVector;
      // Symbol: drake::schema::ToSymbolic
      struct /* ToSymbolic */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_var =
R"""(Like Distribution∷ToSymbolic, but on a DistributionVariant instead.)""";
        // Source: drake/common/schema/stochastic.h
        const char* doc_1args_vec =
R"""(Like Distribution∷ToSymbolic, but elementwise over a collection of
possibly-heterogenous DistributionVariant instead.)""";
      } ToSymbolic;
      // Symbol: drake::schema::Transform
      struct /* Transform */ {
        // Source: drake/common/schema/transform.h
        const char* doc =
R"""(A specification for a 3d rotation and translation, optionally with
respect to a base frame.

For an overview of configuring stochastic transforms, see
schema_transform and schema_stochastic.

See implementing_serialize "Implementing Serialize" for implementation
details, especially the unusually public member fields.)""";
        // Symbol: drake::schema::Transform::GetDeterministicValue
        struct /* GetDeterministicValue */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(If this is deterministic, retrieves its value.

Raises:
    RuntimeError if this is not fully deterministic.)""";
        } GetDeterministicValue;
        // Symbol: drake::schema::Transform::IsDeterministic
        struct /* IsDeterministic */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Returns true iff this is fully deterministic.)""";
        } IsDeterministic;
        // Symbol: drake::schema::Transform::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Returns the mean of this rotation. If this is deterministic, the
result is the same as GetDeterministicValue. If this is random, note
that the mean here is simply defined as setting all of the random
variables individually to their mean. Various other measures of the
resulting RigidTransform (e.g., the distribution of one of the Euler
angles) may not necessarily match that measure on the returned value.)""";
        } Mean;
        // Symbol: drake::schema::Transform::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Samples this Transform. If this is deterministic, the result is the
same as GetDeterministicValue.)""";
        } Sample;
        // Symbol: drake::schema::Transform::SampleAsTransform
        struct /* SampleAsTransform */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Samples this Transform; the returned value is deterministic and has
the same base frame.)""";
        } SampleAsTransform;
        // Symbol: drake::schema::Transform::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/transform.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::Transform::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Returns the symbolic form of this rotation. If this is deterministic,
the result will contain no variables. If this is random, the result
will contain one or more random variables, based on the distributions
in use.)""";
        } ToSymbolic;
        // Symbol: drake::schema::Transform::Transform
        struct /* ctor */ {
          // Source: drake/common/schema/transform.h
          const char* doc_0args = R"""(Constructs the Identity transform.)""";
          // Source: drake/common/schema/transform.h
          const char* doc_1args = R"""(Constructs the given transform.)""";
        } ctor;
        // Symbol: drake::schema::Transform::base_frame
        struct /* base_frame */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(An optional base frame name for this transform. When left unspecified,
the default depends on the semantics of the enclosing struct.)""";
        } base_frame;
        // Symbol: drake::schema::Transform::rotation
        struct /* rotation */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(A variant that allows for several ways to specify a rotation.)""";
        } rotation;
        // Symbol: drake::schema::Transform::set_rotation_rpy_deg
        struct /* set_rotation_rpy_deg */ {
          // Source: drake/common/schema/transform.h
          const char* doc =
R"""(Sets the rotation field to the given deterministic RPY, in degrees.)""";
        } set_rotation_rpy_deg;
        // Symbol: drake::schema::Transform::translation
        struct /* translation */ {
          // Source: drake/common/schema/transform.h
          const char* doc = R"""(A translation vector, in meters.)""";
        } translation;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("base_frame", base_frame.doc),
            std::make_pair("rotation", rotation.doc),
            std::make_pair("translation", translation.doc),
          };
        }
      } Transform;
      // Symbol: drake::schema::Uniform
      struct /* Uniform */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(A uniform distribution with ``min`` inclusive and ``max`` exclusive.)""";
        // Symbol: drake::schema::Uniform::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::Uniform::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::Uniform::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::Uniform::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::Uniform::Uniform
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::Uniform::max
        struct /* max */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } max;
        // Symbol: drake::schema::Uniform::min
        struct /* min */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } min;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("max", max.doc),
            std::make_pair("min", min.doc),
          };
        }
      } Uniform;
      // Symbol: drake::schema::UniformDiscrete
      struct /* UniformDiscrete */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(Chooses from among discrete ``values`` with equal probability.)""";
        // Symbol: drake::schema::UniformDiscrete::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::UniformDiscrete::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::UniformDiscrete::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::UniformDiscrete::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::UniformDiscrete::UniformDiscrete
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::UniformDiscrete::values
        struct /* values */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } values;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("values", values.doc),
          };
        }
      } UniformDiscrete;
      // Symbol: drake::schema::UniformVector
      struct /* UniformVector */ {
        // Source: drake/common/schema/stochastic.h
        const char* doc =
R"""(A uniform distribution with vector ``min`` inclusive and vector
``max`` exclusive.

Template parameter ``Size``:
    rows at compile time (max 6) or else Eigen∷Dynamic.)""";
        // Symbol: drake::schema::UniformVector::Mean
        struct /* Mean */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Mean;
        // Symbol: drake::schema::UniformVector::Sample
        struct /* Sample */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Sample;
        // Symbol: drake::schema::UniformVector::Serialize
        struct /* Serialize */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::schema::UniformVector::ToSymbolic
        struct /* ToSymbolic */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ToSymbolic;
        // Symbol: drake::schema::UniformVector::UniformVector<Size>
        struct /* ctor */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::schema::UniformVector::max
        struct /* max */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } max;
        // Symbol: drake::schema::UniformVector::min
        struct /* min */ {
          // Source: drake/common/schema/stochastic.h
          const char* doc = R"""()""";
        } min;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("max", max.doc),
            std::make_pair("min", min.doc),
          };
        }
      } UniformVector;
    } schema;
  } drake;
} pydrake_doc_common_schema;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

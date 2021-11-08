#pragma once

#include <optional>
#include <string>

#include "drake/common/name_value.h"
#include "drake/common/schema/rotation.h"
#include "drake/common/schema/stochastic.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace schema {

/** @defgroup schema_transform Configuring transforms
@ingroup stochastic_systems
@{

This page describes how to use classes such as schema::Rotation and
schema::Transform to denote stochastic quantities, as a bridge between loading
a scenario specification and populating the corresponding math::RigidTransform
quantities.

The broader concepts are discussed at @ref schema_stochastic. Here, we cover
details related to rotations and transforms in particular.

We'll explain uses of schema::Rotation and schema::Transform using their
matching YAML syntax as parsed by yaml::LoadYamlFile.

# Rotations

This shows the syntax for a schema::Rotation.

When no details are given, the default rotation is the identity matrix:
```
rotation: {}
```

For clarity, you may also specify `Identity` variant tag explicitly.
This version and the above version have exactly the same effect:

```
rotation: !Identity {}
```

To specify roll, pitch, yaw angles using math::RollPitchYaw conventions, use
`Rpy` as the variant tag:

```
rotation: !Rpy
  deg: [10.0, 20.0, 30.0]
```

To specify a rotation angle and axis in the sense of Eigen::AngleAxis, use
`AngleAxis` as the variant tag:

```
rotation: !AngleAxis
  angle_deg: 10.0
  axis: [0.0, 1.0, 0.0]
```

You may also use YAML's flow style to fit everything onto a single line.
These one-line spellings are the equivalent to those above.

```
rotation: !Rpy { deg: [10.0, 20.0, 30.0] }
```

```
rotation: !AngleAxis { angle_deg: 10.0, axis: [0.0, 1.0, 0.0] }
```

## Stochastic Rotations

To specify a stochastic rotation sampled from a uniform distribution over
SO(3):

```
rotation: !Uniform {}
```

The other available representations also accept stochastic distributions for
their values:

```
rotation: !Rpy
  deg: !UniformVector
    min: [ 0.0, 10.0, 20.0]
    max: [30.0, 40.0, 50.0]
```

Or:

```
rotation: !AngleAxis
  angle_deg: !Uniform
    min: 8.0
    max: 10.0
  axis: !UniformVector
    min: [0.0, 0.9, 0.0]
    max: [0.1, 1.0, 0.0]
```

For an explanation of `!Uniform`, `!UniformVector`, and other available options
(%Gaussian, etc.) for scalar and vector quantities, see @ref schema_stochastic.

# Transforms

This shows the syntax for a schema::Transform.  A transform is merely a
translation and rotation, optionally with a some given string as a base_frame.

```
transform:
  translation: [1.0, 2.0, 3.0]
  rotation: !Rpy { deg: [10.0, 20.0, 30.0] }
```

Or:

```
transform:
  base_frame: foo
  translation: [0.0, 0.0, 1.0]
  rotation: !Identity {}
```

## Stochastic Transforms

Either or both of the rotational or translation component can be stochastic:

```
transform:
  translation: !UniformVector
    min: [-1.0, -1.0, -1.0]
    max: [ 1.0,  1.0,  1.0]
  rotation: !Uniform {}
```

For an explanation of `!Uniform`, `!UniformVector`, and other available options
(%Gaussian, etc.) for scalar and vector quantities, see @ref schema_stochastic.

@} */

/// A specification for a 3d rotation and translation, optionally with respect
/// to a base frame.
///
/// For an overview of configuring stochastic transforms, see
/// @ref schema_transform and @ref schema_stochastic.
///
/// See @ref serialize_tips for implementation details, especially the
/// unusually public member fields.
class Transform {
 public:
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
  /// @throws std::exception if this is not fully deterministic.
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
  math::RigidTransformd Sample(RandomGenerator* generator) const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(base_frame));
    a->Visit(DRAKE_NVP(translation));
    a->Visit(MakeNameValue("rotation", &rotation.value));
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

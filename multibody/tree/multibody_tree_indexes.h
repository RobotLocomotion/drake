#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {

namespace internal {

/// Type used to identify mobilizers by index in a multibody tree system.
using MobilizerIndex = TypeSafeIndex<class MobilizerTag>;

/// Type used to identify tree nodes by index within a multibody tree system.
using BodyNodeIndex = TypeSafeIndex<class BodyNodeTag>;

}  // namespace internal

// N.B. To simplify checking binding coverage, please ensure these symbols
// are defined in `multibody_tree_py.cc` in the same order.

/// Type used to identify frames by index in a multibody tree system.
using FrameIndex = TypeSafeIndex<class FrameTag>;

/// Type used to identify bodies by index in a multibody tree system.
using BodyIndex = TypeSafeIndex<class BodyTag>;

/// WARNING: This will be removed on or around 2019/03/01.
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
typedef internal::MobilizerIndex MobilizerIndex;

/// WARNING: This will be removed on or around 2019/03/01.
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
typedef internal::BodyNodeIndex BodyNodeIndex;

/// Type used to identify force elements by index within a multibody tree
/// system.
using ForceElementIndex = TypeSafeIndex<class ForceElementTag>;

/// Type used to identify joints by index within a multibody tree system.
using JointIndex = TypeSafeIndex<class JointElementTag>;

/// Type used to identify actuators by index within a multibody tree system.
using JointActuatorIndex = TypeSafeIndex<class JointActuatorElementTag>;

/// Type used to identify model instances by index within a multibody
/// tree system.
using ModelInstanceIndex = TypeSafeIndex<class ModelInstanceTag>;

// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of BodyIndex
// instead of using a static variable.
/// For every MultibodyTree the **world** body _always_ has this unique index
/// and it is always zero.
inline BodyIndex world_index() { return BodyIndex(0); }

}  // namespace multibody
}  // namespace drake

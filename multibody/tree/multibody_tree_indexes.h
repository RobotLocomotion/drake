#pragma once

#include "drake/common/identifier.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {

namespace internal {

// Type used to identify any quantity associated with a "mobilized body"
// (abbreviated "mobod"), which is a depth-first numbered node of the spanning
// forest used to model a multibody system. This includes BodyNodes, Mobilizers,
// and associated computed quantities such as their accelerations.
using MobodIndex = TypeSafeIndex<class MobodTag>;

// Type used to identify a topological tree within the "forest" of a multibody
// system.
using TreeIndex = TypeSafeIndex<class TreeTag>;

// World is always modeled as the 0th mobilized body.
inline MobodIndex world_mobod_index() {
  return MobodIndex(0);
}

}  // namespace internal

// N.B. To simplify checking binding coverage, please ensure these symbols
// are defined in `tree_py.cc` in the same order.

/// Type used to identify frames by index in a multibody plant.
using FrameIndex = TypeSafeIndex<class FrameTag>;

/// Type used to identify RigidBodies (a.k.a. Links) by index in a multibody
/// plant. Interchangeable with LinkIndex.
using BodyIndex = TypeSafeIndex<class RigidBodyTag>;

/// Type used to identify force elements by index within a multibody plant.
using ForceElementIndex = TypeSafeIndex<class ForceElementTag>;

/// Type used to identify joints by index within a multibody plant.
using JointIndex = TypeSafeIndex<class JointElementTag>;

/// Type used to identify joints by ordinal within a multibody plant.
using JointOrdinal = TypeSafeIndex<class JointOrdinalTag>;

/// Type used to identify actuators by index within a multibody plant.
using JointActuatorIndex = TypeSafeIndex<class JointActuatorElementTag>;

/// Type used to identify constraint by id within a multibody plant.
using MultibodyConstraintId = Identifier<class ConstraintTag>;

/// Type used to identify model instances by index within a multibody plant.
using ModelInstanceIndex = TypeSafeIndex<class ModelInstanceTag>;

// TODO(xuchenhan-tri): Originally, DeformableBodyId is used to uniquely
// identify deformable bodies in a DeformableModel and DeformableBodyIndex is
// used only internally. The intention was to allow easy deletion of deformable
// bodies. However, we have since adopted the "ordinal" approach to allow
// deletion of multibody elements. We should remove DeformableBodyId altogether
// and consistently use DeformableBodyIndex instead.
/// Type used to identify a deformable body by id within a multibody plant.
using DeformableBodyId = Identifier<class DeformableBodyTag>;

/// Type used to identify a deformable body by index within a multibody plant.
using DeformableBodyIndex = TypeSafeIndex<class DeformableBodyTag>;

// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of pre-defined indices
// instead of using a static variable.

/// For every MultibodyPlant the **world** body _always_ has this unique index
/// and it is always zero.
inline BodyIndex world_index() {
  return BodyIndex(0);
}

/// For every MultibodyPlant the **world** frame _always_ has this unique index
/// and it is always zero.
inline FrameIndex world_frame_index() {
  return FrameIndex(0);
}

/// Returns the model instance containing the *world* body.  For
/// every MultibodyPlant the **world** body _always_ has this unique
/// model instance and it is always zero (as described in #3088).
inline ModelInstanceIndex world_model_instance() {
  return ModelInstanceIndex(0);
}

/// Returns the model instance which contains all tree elements with
/// no explicit model instance specified.
inline ModelInstanceIndex default_model_instance() {
  return ModelInstanceIndex(1);
}

}  // namespace multibody
}  // namespace drake

#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

// TODO(russt): We should also provide SpatialVelocitySelector, and
// SpatialAccelerationSelector, to carve up the other std::vector output ports
// from MultibodyPlant.  While it is tempting to template<template> this class,
// or use type erasure, that gets into the "sharp corners" discussed in
// https://github.com/RobotLocomotion/drake/issues/16923 .

/**
Makes a single element from a std::vector<RigidTransform<T>> available on the
single output port. This can be used, for instance, to extract a single pose
from the `body_poses` output port of MultibodyPlant.

@system
name: RigidTransformSelector
input_ports:
- vector
output_ports:
- element
@endsystem

Note: In Drake parlance, a "multiplexer" combines multiple input ports into a
single output port, a "demultiplexer" extracts a single input port into multiple
output ports, and a "selector" extracts a portion of the input port into a
single output port.

@tparam_default_scalar
@ingroup multibody_systems
*/
template <typename T>
class RigidTransformSelector final
    : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidTransformSelector)

  /**
  Constructor.
  @param num_inputs Number of input ports to be added.
  */
  explicit RigidTransformSelector(int index);

  /** Returns the index into the input vector that will be selected. */
  int index() const { return index_; }

  /**
  Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  */
  template <typename U>
  explicit RigidTransformSelector(
      const RigidTransformSelector<U>& other);

 private:
  using ValueType = math::RigidTransform<T>;
  using ListType = std::vector<ValueType>;

  // This is the calculator for the output port.
  void CalcOutput(
      const systems::Context<T>& context, ValueType* output) const;

  int index_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RigidTransformSelector)

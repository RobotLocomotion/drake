#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
Combines multiple lists of externally applied spatial forces.

@system
name: ExternallyAppliedSpatialForceListMultiplexer
input_ports:
- u0
- ...
- u(N-1)
output_ports:
- combined
@endsystem

@tparam default_scalar
*/
template <typename T>
class ExternallyAppliedSpatialForceListMultiplexer final
    : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExternallyAppliedSpatialForceListMultiplexer)

  using ValueType = ExternallyAppliedSpatialForce<T>;
  using ListType = std::vector<ValueType>;

  /**
  Constructor.
  @param num_inputs Number of input ports to be added.
  */
  explicit ExternallyAppliedSpatialForceListMultiplexer(int num_inputs);

  /**
  Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  */
  template <typename U>
  explicit ExternallyAppliedSpatialForceListMultiplexer(
      const ExternallyAppliedSpatialForceListMultiplexer<U>& other);

 private:
  // This is the calculator for the output port.
  void CombineInputsToOutput(
      const systems::Context<T>& context, ListType* output) const;
};

}  // namespace multibody
}  // namespace drake

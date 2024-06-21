#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
Concatenates multiple std::vector<>'s of ExternallyAppliedSpatialForce<T>.

@system
name: ExternallyAppliedSpatialForceMultiplexer
input_ports:
- u0
- ...
- u(N-1)
output_ports:
- y0
@endsystem

@tparam_default_scalar
*/
template <typename T>
class ExternallyAppliedSpatialForceMultiplexer final
    : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExternallyAppliedSpatialForceMultiplexer);

  /**
  Constructor.
  @param num_inputs Number of input ports to be added.
  */
  explicit ExternallyAppliedSpatialForceMultiplexer(int num_inputs);

  /**
  Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  */
  template <typename U>
  explicit ExternallyAppliedSpatialForceMultiplexer(
      const ExternallyAppliedSpatialForceMultiplexer<U>& other);

  ~ExternallyAppliedSpatialForceMultiplexer() final;

 private:
  using ValueType = ExternallyAppliedSpatialForce<T>;
  using ListType = std::vector<ValueType>;

  // This is the calculator for the output port.
  void CombineInputsToOutput(const systems::Context<T>& context,
                             ListType* output) const;
};

}  // namespace multibody
}  // namespace drake

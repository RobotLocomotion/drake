#pragma once

#include <string>
#include <tuple>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace systems {
namespace lcm {

/** %LcmScopeSystem provides the ability to convert any vector output port to a
simple LCM message and publish that message periodically. The intention is to
instrument complex diagrams using external tools like `lcm-spy`.

@ingroup message_passing */
class LcmScopeSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmScopeSystem)

  /** (Advanced.) Most users will use AddToBuilder instead of this constructor.
  On its own, this constructor does not publish anything.

  Creates a system with one input port and one output port. The input accepts
  a vector of the given `size`. The output produces an lcmt_scope message. */
  explicit LcmScopeSystem(int size);

  /** Adds an LcmScopeSystem and LcmPublisherSystem to the given `builder`.

  @param[in] lcm A pointer to the LCM subsystem to use, which must remain valid
  for the lifetime of `builder`. This will typically be an instance of an
  LcmInterfaceSystem that's already been added to the `builder`.

  @param[in] signal The output port to be scoped.  Must be an OutputPort of a
  system that's already been added to the `builder`.

  @param[in] channel The LCM channel on which to publish.

  @param[in] publish_period Specifies how often messages will be published.
  If the period is zero, the LcmPublisherSystem will publish every step. */
  static std::tuple<LcmScopeSystem*, LcmPublisherSystem*> AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      drake::lcm::DrakeLcmInterface* lcm,
      const OutputPort<double>& signal,
      const std::string& channel,
      double publish_period);
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake

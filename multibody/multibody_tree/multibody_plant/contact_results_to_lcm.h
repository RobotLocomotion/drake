#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/** A System that encodes ContactResults into a lcmt_contact_results_for_viz
 message. It has a single input port with type ContactResults<T> and a single
 output port with lcmt_contact_results_for_viz.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
   - double
   - AutoDiffXd

 They are already available to link against in the containing library. No other
 values for T are currently supported.
 */
template <typename T>
class ContactResultsToLcmSystem final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /** Constructs a ContactResultsToLcmSystem.
   @param plant The MultibodyPlant that the ContactResults are generated from.
   @pre The `plant` must be finalized already. The input port of this system
        must be connected to the corresponding output port of `plant`.  */
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>& plant);

  /** Scalar-converting copy constructor.  */
  template <typename U>
  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>& other)
      : systems::LeafSystem<T>(), body_names_(other.body_names_) {}

  const systems::InputPortDescriptor<T>& get_contact_result_input_port() const;
  const systems::OutputPort<T>& get_lcm_message_output_port() const;

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class ContactResultsToLcmSystem;

  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  // Named indices for the i/o ports.
  static constexpr int contact_result_input_port_index_{0};
  static constexpr int message_output_port_index_{0};

  // A mapping from body index values to body names.
  std::vector<std::string> body_names_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

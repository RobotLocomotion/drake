#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/**
 * A System that encodes ContactResults into a lcmt_contact_results_for_viz
 * message. It has a single input port with type ContactResults<T> and a
 * single output port with lcmt_contact_results_for_viz.
 */
template <typename T>
class ContactResultsToLcmSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /**
   * Constructs a ContactResultsToLcmSystem.
   * @param tree, The RigidBodyTree that the ContactResults are generated with,
   * which should be returned by RigidBodyPlant's get_rigid_body_tree(). The
   * life span of @p tree needs to be longer than this instance.
   */
  explicit ContactResultsToLcmSystem(const MultibodyPlant<T>& plant);

 private:
  void CalcLcmContactOutput(const systems::Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  const MultibodyPlant<T>& plant_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

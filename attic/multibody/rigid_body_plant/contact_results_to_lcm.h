#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * A System that encodes ContactResults into a lcmt_contact_results_for_viz
 * message. It has a single input port with type ContactResults<T> and a
 * single output port with lcmt_contact_results_for_viz.
 *
 * @ingroup visualization
 */
template <typename T>
class ContactResultsToLcmSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToLcmSystem)

  /**
   * Constructs a ContactResultsToLcmSystem.
   * @param tree, The RigidBodyTree that the ContactResults are generated with,
   * which should be returned by RigidBodyPlant's get_rigid_body_tree(). The
   * life span of @p tree needs to be longer than this instance.
   */
  explicit ContactResultsToLcmSystem(const RigidBodyTree<T>& tree);

 private:
  void CalcLcmContactOutput(const Context<T>& context,
                            lcmt_contact_results_for_viz* output) const;

  const RigidBodyTree<T>& tree_;
};

}  // namespace systems
}  // namespace drake

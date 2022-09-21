#pragma once

// Note: the user should not include this header in their code. This header is
// created for internal use only.

#include <memory>
#include <unordered_set>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace c_iris {
namespace internal {
/**
 * In the changed-root tree (where link A is treated as the root of the tree,
 * instead of the world link), we need to re-compute the parent-child
 * relationship. Hence ChangedRootBody stores the updated parent, children,
 * and the mobilizer between the body and the parent.
 */
struct ChangedRootBody {
  ChangedRootBody(
      multibody::BodyIndex m_body_index, const ChangedRootBody* const m_parent,
      const multibody::internal::Mobilizer<double>* const m_mobilizer)
      : body_index{m_body_index}, parent{m_parent}, mobilizer{m_mobilizer} {}
  // The index of this body in the original tree.
  multibody::BodyIndex body_index;
  // The parent of this body in the changed-root tree.
  const ChangedRootBody* const parent;
  // The children of this body in the changed-root tree.
  std::vector<std::unique_ptr<ChangedRootBody>> children;
  // The mobilizer between this body and the parent in the changed-root tree.
  const multibody::internal::Mobilizer<double>* const mobilizer;
};

/**
 * Given a multibody plant, rebuild its kinematic tree at a specified root.
 * TODO(hongkai.dai): Consider to use the more general solution once issue
 * #17249 is resolved.
 */
void ChangeKinematicTreeRoot(const multibody::MultibodyPlant<double>& plant,
                             ChangedRootBody* root);

/**
 * Find and add all the children to the body in the changed-root tree.
 * @param visited Keeps the indices of the body in the changed-root tree that
 * have been visited, in the process of building the changed-root tree.
 */
void AddChildrenToChangedRootBody(
    const multibody::MultibodyPlant<double>& plant, ChangedRootBody* body,
    std::unordered_set<multibody::BodyIndex>* visited);

/**
 * Find the path on the kinematic tree from start to the end.
 */
std::vector<multibody::BodyIndex> FindPath(
    const multibody::MultibodyPlant<double>& plant, multibody::BodyIndex start,
    multibody::BodyIndex end);

/**
 * Find all the mobilizer on the path from start to the end.
 */
std::vector<multibody::internal::MobilizerIndex> FindMobilizersOnPath(
    const multibody::MultibodyPlant<double>& plant, multibody::BodyIndex start,
    multibody::BodyIndex end);

/**
 * Find the body in the middle of the kinematic chain that goes from the start
 * to the end. Notice that we ignore the welded joint, and only count revolute
 * joint as one step along the chain, we throw an error when hitting non-revolue
 * / weld mobilizers.
 */
multibody::BodyIndex FindBodyInTheMiddleOfChain(
    const multibody::MultibodyPlant<double>& plant, multibody::BodyIndex start,
    multibody::BodyIndex end);
}  // namespace internal
}  // namespace c_iris
}  // namespace multibody
}  // namespace drake

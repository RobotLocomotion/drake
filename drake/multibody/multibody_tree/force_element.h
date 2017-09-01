#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {


template <typename T>
class ForceElement : public
                     MultibodyTreeElement<ForceElement<T>, ForceElementIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ForceElement)

  /// The minimum amount of information that we need to define a %Mobilizer is
  /// the knowledge of the inboard and outboard frames it connects.
  /// Subclasses of %Mobilizer are therefore required to provide this
  /// information in their respective constructors.
  /// @throws std::runtime_error if `inboard_frame` and `outboard_frame`
  /// reference the same frame object.
  ForceElement() {}

  void CalcAndAddForceContribution(const MultibodyTreeContext<T>& context,
                                   const PositionKinematicsCache<T>& pc,
                                   const VelocityKinematicsCache<T>& vc,
                                   std::vector<SpatialForce<T>>* F_B_W,
                                   Eigen::Ref<VectorX<T>> tau) const {
    DRAKE_DEMAND(F_B_W != nullptr);
    DRAKE_DEMAND(static_cast<int>(F_B_W->size()) ==
        this->get_parent_tree().get_num_bodies());
    DRAKE_DEMAND(tau.size() == this->get_parent_tree().get_num_velocities());
    DoCalcAndAddForceContribution(context, pc, vc, F_B_W, tau);
  }

  /// @cond
  // For internal use only.
  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> CloneToScalar(
  const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }
  /// @endcond

 protected:

  virtual void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      Eigen::Ref<VectorX<T>> tau) const = 0;

  /// @name Methods to make a clone templated on different scalar types.
  /// @{

  /// Clones this %ForceElement (templated on T) to a mobilizer templated on
  /// `double`.
  virtual std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %ForceElement (templated on T) to a mobilizer templated on
  /// AutoDiffXd.
  virtual std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

 private:
  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each force element retrieves its
  // topology from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {}
};

}  // namespace multibody
}  // namespace drake

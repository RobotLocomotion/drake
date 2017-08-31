#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
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

 protected:

  virtual void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      Eigen::Ref<VectorX<T>> tau) const = 0;

#if 0
  /// NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  /// be created. This method is mostly intended to be called by
  /// MultibodyTree::CloneToScalar(). Most users should not call this clone
  /// method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }

  /// For MultibodyTree internal use only.
  virtual std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node,
      const Body<T>* body, const Mobilizer<T>* mobilizer) const = 0;

 protected:
  /// @name Methods to make a clone templated on different scalar types.
  ///
  /// The only const argument to these methods is the new MultibodyTree clone
  /// under construction, which is required to already own the clones of the
  /// inboard and outboard frames of the mobilizer being cloned.
  /// @{

  /// Clones this %Mobilizer (templated on T) to a mobilizer templated on
  /// `double`.
  /// @pre Inboard and outboard frames for this mobilizer already have a clone
  /// in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Mobilizer (templated on T) to a mobilizer templated on
  /// AutoDiffXd.
  /// @pre Inboard and outboard frames for this mobilizer already have a clone
  /// in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}
#endif

 private:
  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each force element retrieves its
  // topology from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {}
};

template <typename T>
class UniformGravityElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityElement)

  UniformGravityElement(Vector3<double> g_W);

  /// Returns the acceleration of gravity vector, expressed in the world frame
  /// W.
  const Vector3<double>& g_W() const { return g_W_; }

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      Eigen::Ref<VectorX<T>> tau) const final;

 private:
  Vector3<double> g_W_;
};

}  // namespace multibody
}  // namespace drake

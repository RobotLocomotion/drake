#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T>
class UniformGravityFieldElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityFieldElement)

  UniformGravityFieldElement(Vector3<double> g_W);

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

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  Vector3<double> g_W_;
};

}  // namespace multibody
}  // namespace drake

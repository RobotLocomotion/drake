#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

/// This ForceElement allows modeling the effect of a uniform gravity field as
/// felt by bodies on the surface of the Earth.
/// This gravity fields acts on all bodies in the MultibodyTree model.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class UniformGravityFieldElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformGravityFieldElement)

  /// Constructs a uniform gravity field element with a strength given by the
  /// acceleration of gravity vector `g_W`, expressed in the world frame W.
  explicit UniformGravityFieldElement(Vector3<double> g_W);

  /// Returns the acceleration of gravity vector, expressed in the world frame
  /// W.
  const Vector3<double>& gravity_vector() const { return g_W_; }

  T CalcPotentialEnergy(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc) const final;

  T CalcConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

  T CalcNonConservativePower(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

 protected:
  void DoCalcAndAddForceContribution(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      EigenPtr<VectorX<T>> tau) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  Vector3<double> g_W_;
};

}  // namespace multibody
}  // namespace drake

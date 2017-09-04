#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/force_element.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// This ForceElement allows to model the effect of a uniform gravity field.
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
class RodElement : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RodElement)

  /// Assumptions:
  ///  1. Body frames are located at their center of mass.
  ///  2. Body frame third axis, d3, goes along its axial direction.
  explicit RodElement(
      const Body<T>& body1, double length1,
      const Body<T>& body2, double length2,
      double B1, double B2, double C, double damping);

  T CalcPotentialEnergy(
      const MultibodyTreeContext<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc) const final;

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
      Eigen::Ref<VectorX<T>> tau) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  BodyIndex body_i_index_;
  double length1_;
  BodyIndex body_ip_index_;
  double length2_;
  // Bending stiffness coefficients.
  double B1_, B2_, C_;
  // Damping coefficeint
  // TODO: look into Rayleigh damping (FEA) and/or
  // Kelvin–Voigt damping: sigma = E * strain + beta * strain_dot.
  double damping_;
  // Length of the rod at i + 1/2.
  double length_ip_;
};

}  // namespace multibody
}  // namespace drake
